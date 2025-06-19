import trimesh
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
import matplotlib.pyplot as plt

def label_flattened_by_original_heights(original_mesh_path, flattened_mesh_path, height_intervals):
    """
    Classifies the faces of the remeshed and plane mesh depending on the Z height in the original mesh.
    
    Args:
        original_mesh_path: Path to the original mesh.
        flattened_mesh_path: Path to the remeshed and plane mesh.
        height_intervals: List of tuples [(min_z, max_z), ...]

    Returns:
        List of plane and remeshed submeshes, one per height range in the original.
    """
    # Load meshes
    original_mesh = trimesh.load(original_mesh_path)
    flattened_mesh = trimesh.load(flattened_mesh_path)

    # Trasform the meshes to numpy arrays
    vertices_orig = np.asarray(original_mesh.vertices)
    vertices_flat = np.asarray(flattened_mesh.vertices)
    triangles_flat = np.asarray(flattened_mesh.triangles)

    # Create KDTree to search the closest Z height.
    kdtree = cKDTree(vertices_orig)
    k = 3
    dists, indexes = kdtree.query(vertices_flat, k=k)
    weights = 1 / (dists + 1e-8)  # avoid division by zero
    weights /= weights.sum(axis=1, keepdims=True)

    # Weighted average height
    z_orig_weighted = np.sum(vertices_orig[indexes][:,:,2] * weights, axis=1)

    # Label with the average Z
    vertex_labels = np.full(len(vertices_flat), -1, dtype=int)
    for i, (zmin, zmax) in enumerate(height_intervals):
        mask = (z_orig_weighted >= zmin) & (z_orig_weighted < zmax)
        vertex_labels[mask] = i

    # Label the triangles if all vertices are in the same Z label
    face_labels = np.full(len(triangles_flat), -1, dtype=int)
    for i, tri in enumerate(triangles_flat):
        vl = vertex_labels[tri]
        counts = np.bincount(vl[vl >= 0])
        if len(counts) > 0:
            face_labels[i] = np.argmax(counts)

    # Create submeshes
    submeshes = []
    for i in range(len(height_intervals)):
        face_mask = face_labels == i
        selected_faces = triangles_flat[face_mask]

        if len(selected_faces) == 0:
            submeshes.append(o3d.geometry.TriangleMesh())
            continue

        # Find the unique vertices used in these faces.
        unique_vertices, inverse_indexes = np.unique(selected_faces.flatten(), return_inverse=True)
        new_vertices = vertices_flat[unique_vertices]
        new_faces = inverse_indexes.reshape((-1, 3))

        # Create new TriangleMesh
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(new_vertices)
        mesh.triangles = o3d.utility.Vector3iVector(new_faces)
        mesh.compute_vertex_normals()

        submeshes.append(mesh)

    return submeshes


def plane_and_remesh(mesh_path, output_path, remesh_size):
    """
    Transforms the original mesh in a plane with an specified vertex size.

    Args:
        mesh_path: The path to the mesh.
        output_path: The path to save the modified mesh.
        remesh_size: The wanted vertex size.

    Returns:
        Saves a plane mesh with the specified vertex size.
    """
    #Load mesh.
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    # Raise error if the mesh doesn't have any vertices.
    if not mesh.has_vertices():
        raise ValueError("The mesh doesn't have any vertices.")

    vertices = np.asarray(mesh.vertices)

    # We set de Z value to 0 to create a plane.
    vertices[:, 2] = 0

    mesh.vertices = o3d.utility.Vector3dVector(vertices)

    mesh.compute_vertex_normals()

    # We use vertex clustering to remesh with the wanting vertex size.
    mesh_modified = mesh.simplify_vertex_clustering(
        voxel_size=remesh_size,
        contraction=o3d.geometry.SimplificationContraction.Average)

    # Save the modified mesh.
    o3d.io.write_triangle_mesh(output_path, mesh_modified)


def create_grid_from_mesh_shapely(mesh_path, csv_path, cell_size=0.10, xlim=None, ylim=None):
    """
    Given a mesh and a cell size, creates a grid with -1 values outside the mesh and 0 inside.

    Args:
        mesh_path: The path to the mesh.
        csv_path: The path to save the grid as a CSV file. 
        cell_size: Size of the cells in the grid.
        xlim: A tuple (min_x, max_x) specifying the horizontal (X-axis) range to focus on. If None, the full X range of the mesh will be used.
        ylim: A tuple (min_y, max_y) specifying the horizontal (Y-axis) range to focus on. If None, the full Y range of the mesh will be used.

    Returns:
        Saves the grid in a CSV file.
    """
    # Load mesh.
    mesh = trimesh.load(mesh_path)
    vertices_2d = mesh.vertices[:, :2]  # We are only interested in X and Y.
    print("Mesh loaded")

    # Bounding box of the area
    min_x, min_y = vertices_2d.min(axis=0)
    max_x, max_y = vertices_2d.max(axis=0)

    if xlim is not None:
        min_x, max_x = xlim
    if ylim is not None:
        min_y, max_y = ylim

    nx = int((max_x - min_x) / cell_size) + 1
    ny = int((max_y - min_y) / cell_size) + 1

    x_coords = min_x + np.arange(nx) * cell_size + cell_size / 2
    y_coords = min_y + np.arange(ny) * cell_size + cell_size / 2
    print("Grid parameters calculated")

    # Create the triangles as shapely polygons
    polygons = [Polygon(vertices_2d[face]) for face in mesh.faces if Polygon(vertices_2d[face]).is_valid]
    print("Polygons created")

    # Join all the triangles in a single shape.
    mesh_area = unary_union(polygons)
    print("Area created")

    # Create the center of each cell.
    xx, yy = np.meshgrid(x_coords, y_coords)
    points = [Point(x, y) for x, y in zip(xx.ravel(), yy.ravel())]
    print("Points created")

    # Verify if each point it's inside the mesh area.
    mask = np.array([mesh_area.contains(p) for p in points]).reshape((ny, nx))
    print("Mask defined")

    # Create the grid
    grid = -np.ones((ny, nx), dtype=int)
    grid[mask] = 0

    # Transforms the data to a pandas Dataframe.
    data = []
    for i in range(ny):
        for j in range(nx):
            if grid[i, j] >= 0:
                data.append({
                    "x": x_coords[j],
                    "y": y_coords[i],
                    "hits": grid[i, j]
                })

    df = pd.DataFrame(data)

    # Saves the grid in a CSV file.
    df.to_csv(csv_path, index=False)

def load_grid(grid_path):
    """
    Given a CSV file, loads the grid.

    Args:
        grid_path: The path to a CSV file.

    Returns:
        grid: The grid.
        x_coords:  Coordenadas x ordenadas.
        y_coords : Coordenadas y ordenadas.
    """
    df = pd.read_csv(grid_path)
        
    x_coords = np.sort(df["x"].unique())
    y_coords = np.sort(df["y"].unique())

    grid_df = df.pivot_table(index="y", columns="x", values="hits", fill_value=-1)

    grid_df = grid_df.reindex(index=y_coords, columns=x_coords)
    grid = grid_df.to_numpy()

    return grid, x_coords, y_coords


def visualize_grid(grid_path, interval=None):
    """
    Given a grid, visualizes it using matplotlib.

    Args:
        grid_path: The path to a CSV file.
        interval: Tuple with the wanted vmin and vmax. In case of None, the interval would be between 0 and maximum.

    Returns:
        A visualization of the specified grid.
    """
    # Load grid
    grid, x_coords, y_coords = load_grid(grid_path)

    # Visualization
    plt.figure(figsize=(8, 8))
    masked_hits = np.ma.masked_where(grid < 0, grid)
    if interval == None:
        interval = (0, np.max(masked_hits))
    plt.imshow(masked_hits, origin='lower',
            extent=[x_coords[0] - 0.05, x_coords[-1] + 0.05,
                    y_coords[0] - 0.05, y_coords[-1] + 0.05],
            cmap='hot',
            vmin=interval[0],
            vmax=interval[1]
        )
    plt.colorbar(label='Muestreos por celda')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Cobertura y densidad de muestreo')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

def grid_coverage_overlap(grid_path):
    """
    Given a grid, calculates the coverage and overlap.

    Args:
        grid_path: The path to a CSV file.

    Returns:
        Prints the info.
    """
    grid, _, _ = load_grid(grid_path)


    covered_cells = np.sum(grid >= 1)
    total_samples = np.sum(grid[grid >= 0])
    total_valid_cells = np.sum(grid >= 0)

    coverage = (covered_cells / total_valid_cells) * 100
    percent_overlap = np.sum(grid > 1) / total_valid_cells * 100

    print(f"Coverage: {coverage:.2f}%")
    print(f"Overlapping: {percent_overlap:.2f}%")

