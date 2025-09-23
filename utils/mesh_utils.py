import trimesh
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, PowerNorm, BoundaryNorm, ListedColormap
from mpl_toolkits.mplot3d import Axes3D

def plot_flattened_with_height_labels(vertices_flat, z_orig_weighted, vertex_labels):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    sc = ax.scatter(vertices_flat[:, 0], vertices_flat[:, 1], vertices_flat[:, 2],
                    c=z_orig_weighted, cmap='viridis', s=1)
    plt.colorbar(sc, ax=ax, label="Estimated original height (Z)")
    ax.set_title("Vertices of the flatten mesh coloured by original height")
    plt.show()

def plot_vertex_labels(vertices_flat, vertex_labels, title="Labels by height"):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    sc = ax.scatter(vertices_flat[:, 0], vertices_flat[:, 1], vertices_flat[:, 2],
                    c=vertex_labels, cmap='tab10', s=1)
    plt.colorbar(sc, ax=ax, label="Height label")
    ax.set_title(title)
    plt.show()

def plot_submeshes(submeshes):
    for i, mesh in enumerate(submeshes):
        vertices = np.asarray(mesh.vertices)
        if vertices.shape[0] == 0:
            continue
        fig = plt.figure(figsize=(6, 4))
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], s=1)
        ax.set_title(f"Submesh for interval #{i}")
        plt.show()

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
    triangles_flat = np.asarray(flattened_mesh.faces)

    vertices_orig_xy = vertices_orig[:, :2]
    vertices_flat_xy = vertices_flat[:, :2]

    # Create KDTree to search the closest Z height.
    kdtree = cKDTree(vertices_orig_xy)
    k = 3
    dists, indexes = kdtree.query(vertices_flat_xy, k=k)
    weights = 1 / (dists + 1e-8)
    weights /= weights.sum(axis=1, keepdims=True)

    z_orig_weighted = np.sum(vertices_orig[indexes][:,:,2] * weights, axis=1)
    #plot_flattened_with_height_labels(vertices_flat, z_orig_weighted, None)

    # Label with the average Z
    vertex_labels = np.full(len(vertices_flat), -1, dtype=int)
    for i, (zmin, zmax) in enumerate(height_intervals):
        mask = (z_orig_weighted >= zmin) & (z_orig_weighted < zmax)
        vertex_labels[mask] = i
    #plot_vertex_labels(vertices_flat, vertex_labels, title="Labels by original height")

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
            continue  # No mesh to add

        # Find unique vertices used in these faces
        unique_vertices, inverse_indexes = np.unique(selected_faces.flatten(), return_inverse=True)
        new_vertices = vertices_flat[unique_vertices]
        new_faces = inverse_indexes.reshape((-1, 3))

        # Create a temporary mesh
        temp_mesh = o3d.geometry.TriangleMesh()
        temp_mesh.vertices = o3d.utility.Vector3dVector(new_vertices)
        temp_mesh.triangles = o3d.utility.Vector3iVector(new_faces)
        temp_mesh.compute_vertex_normals()

        # Detect connected components
        triangle_clusters, cluster_n_triangles, _ = temp_mesh.cluster_connected_triangles()
        triangle_clusters = np.asarray(triangle_clusters)

        n_clusters = triangle_clusters.max() + 1

        # Create submesh for each cluster
        for cluster_idx in range(n_clusters):
            cluster_mask = triangle_clusters == cluster_idx
            cluster_faces = np.asarray(temp_mesh.triangles)[cluster_mask]

            if len(cluster_faces) == 0:
                continue

            # Find unique vertices in this cluster
            unique_cluster_vertices, cluster_inverse = np.unique(cluster_faces.flatten(), return_inverse=True)
            cluster_vertices = np.asarray(temp_mesh.vertices)[unique_cluster_vertices]
            cluster_faces = cluster_inverse.reshape((-1, 3))

            # Build final mesh
            cluster_mesh = o3d.geometry.TriangleMesh()
            cluster_mesh.vertices = o3d.utility.Vector3dVector(cluster_vertices)
            cluster_mesh.triangles = o3d.utility.Vector3iVector(cluster_faces)
            cluster_mesh.compute_vertex_normals()

            submeshes.append(cluster_mesh)

    #plot_submeshes(submeshes)

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

def open3d_to_trimesh(o3d_mesh):
    """
    Converts an Open3D mesh to a Trimesh mesh.

    Args:
        o3d_mesh: The Open3D mesh.

    Returns:
        The equivalent Trimesh mesh.
    """
    vertices = np.asarray(o3d_mesh.vertices)
    faces = np.asarray(o3d_mesh.triangles)
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def get_shared_edges_by_position(mesh1: trimesh.Trimesh, mesh2: trimesh.Trimesh, mesh_original: trimesh.Trimesh, tol=1e-6):
    """
    Find shared edges between two meshes by comparing the 3D positions of the vertices.
    Use the original mesh to calculate the inclination relative to the Z axis of the faces associated with those edges.
    Returns a set of frozensets, where each frozenset contains two tuples (x, y, z).

    Args:
        mesh1: The first mesh.
        mesh2: The second mesh.
        mesh_original: The original mesh to calculate the inclination.
        tol: Tolerance for considering vertices as identical.
    Returns:
        blocked_edges: Set of edges that should be blocked (shared edges except the door edge).
        door_edges: Set with the edge that should be the door (the one on the face with the smallest slope on the Z axis).
    """

    def round_vertex(v):
        """
        Rounds the vertex coordinates to a specified tolerance.
        This helps in comparing vertices that are very close to each other.

        Args:
            v: The vertex to round.

        Returns:
            The rounded vertex as a tuple.
        """
        return tuple(np.round(v, decimals=int(-np.log10(tol))))

    def get_edges_by_position(mesh):
        """
        Creates a mapping of edges based on the rounded positions of their vertices.

        Args:
            mesh: The mesh to process.

        Returns:
            A dictionary mapping frozensets of rounded vertex positions to edge indices.
        """
        vertices = mesh.vertices
        edges = mesh.edges_unique

        edge_map = dict()
        for edge in edges:
            v0 = round_vertex(vertices[edge[0]])
            v1 = round_vertex(vertices[edge[1]])
            key = frozenset([v0, v1])
            edge_map[key] = edge
        return edge_map
        

    # Create KDTree in 2D (X, Y) for the original mesh
    original_xy = mesh_original.vertices[:, :2]
    original_tree = cKDTree(original_xy)

    def find_faces_for_edge_in_original_soft(edge_key, k=10):
        """
        Finds faces in the original mesh that contain vertices close to the edge defined by edge_key.
        Uses a soft approach by looking for the k nearest vertices in the original mesh.

        Args:
            edge_key: A frozenset containing two vertex positions defining the edge.
            k: Number of nearest neighbors to consider.

        Returns:
            A list of face indices in the original mesh that contain vertices close to the edge.
        """
        v0, v1 = list(edge_key)
        v0_xy = np.array(v0[:2])
        v1_xy = np.array(v1[:2])

        # Get k nearest neighbors in XY (without filtering by distance)
        radius = np.linalg.norm(v0_xy - v1_xy) * 0.6  # Radius proportional to the length of the edge

        idxs_v0 = original_tree.query_ball_point(v0_xy, r=radius)
        idxs_v1 = original_tree.query_ball_point(v1_xy, r=radius)


        # Ensure that they are two-dimensional arrays
        idxs_v0 = np.atleast_1d(idxs_v0)
        idxs_v1 = np.atleast_1d(idxs_v1)
        
        # Search for faces that contain any of the vertices
        faces_v0 = np.where(np.isin(mesh_original.faces, idxs_v0).any(axis=1))[0]
        faces_v1 = np.where(np.isin(mesh_original.faces, idxs_v1).any(axis=1))[0]

        # IIntersection: faces containing at least one vertex of v0 and one of v1
        common_faces = np.intersect1d(faces_v0, faces_v1)

        return common_faces



    def min_inclination_z(shared_keys):
        """
        Finds the edge among shared_keys that is on the face with the minimum inclination relative to the Z axis.

        Args:
            shared_keys: Set of frozensets representing shared edges.

        Returns:
            The edge (as a frozenset) with the minimum inclination.
        """
        min_inclination = float('inf')
        min_edge = None

        for edge_key in shared_keys:
            inclinations = []

            faces = find_faces_for_edge_in_original_soft(edge_key, k=10)
            for face_idx in faces:
                normal = mesh_original.face_normals[face_idx]
                inclination = abs(normal[2])  # Z component of the normal
                inclinations.append(inclination)

            if inclinations:
                min_face_incl = min(inclinations)
                if min_face_incl < min_inclination:
                    min_inclination = min_face_incl
                    min_edge = edge_key

        return min_edge


    # Obtain edge maps by position
    edges1 = get_edges_by_position(mesh1)
    edges2 = get_edges_by_position(mesh2)

    shared_keys = set(edges1.keys()) & set(edges2.keys())
    print(f"Total shared edges found: {len(shared_keys)}")

    if not shared_keys:
        print(shared_keys)
        return shared_keys, shared_keys

    # Select the edge to be removed (the one most perpendicular to the Z axis in the original mesh).
    edge_to_remove = min_inclination_z(shared_keys)

    if edge_to_remove:
        shared_keys.remove(edge_to_remove)

    blocked_edges = shared_keys
    door_edges = {edge_to_remove} if edge_to_remove else set()

    return blocked_edges, door_edges

def save_shared_edges_to_txt(shared_edges, filename="shared_edges.txt"):
    """
    Saves the shared edges to a text file.

    Args:
        shared_edges: Set of edges to save.
        filename: The name of the output text file.
    """
    with open(filename, "w") as f:
        for edge in shared_edges:
            v1, v2 = list(edge)

            # Format to text with clean decimals
            v1_str = f"({v1[0]:.6f}, {v1[1]:.6f}, {v1[2]:.6f})"
            v2_str = f"({v2[0]:.6f}, {v2[1]:.6f}, {v2[2]:.6f})"

            f.write(f"{v1_str} - {v2_str}\n")

    print(f"Data stored in {filename}")

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
        x_coords:  Ordered x-coordinates.
        y_coords : Ordered y-coordinates.
    """
    df = pd.read_csv(grid_path)
        
    x_coords = np.sort(df["x"].unique())
    y_coords = np.sort(df["y"].unique())

    grid_df = df.pivot_table(index="y", columns="x", values="hits", fill_value=-1)

    grid_df = grid_df.reindex(index=y_coords, columns=x_coords)
    grid = grid_df.to_numpy()

    return grid, x_coords, y_coords


def visualize_grid(grid_path, interval=None, paths=None):
    """
    Given a grid, visualizes it using matplotlib.

    Args:
        grid_path: The path to a CSV file.
        interval: Tuple with the wanted vmin and vmax. In case of None, the interval would be between 0 and maximum.
        paths: List of tuples (x, y) of paths to be displayed on top of the grid.

    Returns:
        A visualization of the specified grid.
    """
    # Load grid
    grid, x_coords, y_coords = load_grid(grid_path)

    # Visualization
    plt.figure(figsize=(40, 30))

    masked_hits = np.ma.masked_where(grid < 0, grid)
    if interval == None:
        interval = (0, np.max(masked_hits))
        max_val = int(np.max(masked_hits))
    else:
        max_val = int(interval[1])
    n_variable = max(0, max_val - 1)  # We use other colors for values 0 and 1.
    blues = plt.get_cmap('Blues', 256)
    blues_dark = blues(np.linspace(0.5, 1.0, n_variable))
    custom_colors = [
        '#7A8096',  # 0 coverage
        '#C4B385'   # 1 coverage
    ]

    # Combine: [0, 1] + darker toner of Blues scheme
    all_colors = custom_colors + list(blues_dark)
    cmap = ListedColormap(all_colors)
    plt.imshow(masked_hits, origin='lower',
            extent=[x_coords[0] - 0.05, x_coords[-1] + 0.05,
                    y_coords[0] - 0.05, y_coords[-1] + 0.05],
            vmin=interval[0],
            vmax=interval[1],
            cmap=cmap
        )
    if not paths == None:
        for x, y in paths:
            plt.plot(x, y, color='#9a5f51', linewidth=0.03, marker='o', label='Path', markersize=2)

    
    for spine in plt.gca().spines.values():
        spine.set_visible(False)

    plt.xticks([])
    plt.yticks([])
    plt.grid(False)
    plt.axis('equal')
    plt.grid(False)
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



def merge_grids(paths, csv_path):
    """    
    Merge grids from CSV files. The grids must be from the same area.

    Args:
        paths (list of str): Paths to CSV.
        csv_path: The path to save the merged grid.

    Returns:
        Saves the merged grid in a CSV file.
    """
    if not paths:
        raise ValueError("The list of paths is empty")

    # Read the grids
    grids = [load_grid(path)[0] for path in paths]

    mask = (grids[0] == -1)
    
    grid_merged = np.zeros_like(grids[0], dtype=int)
    
    for grid in grids:
        grid_merged += np.where(grid == -1, 0, grid).astype(int)  # If grid[i] == -1 -> sum 0, if grid[i] != -1, sum the original value.

    # We restore the -1
    grid_merged[mask] = -1

    ny, nx = grid_merged.shape
    first_df = pd.read_csv(paths[0])
    x_coords = sorted(first_df["x"].unique())
    y_coords = sorted(first_df["y"].unique())

    # Transforms the data to a pandas Dataframe.
    data = []
    for i in range(ny):
        for j in range(nx):
            if grid_merged[i, j] >= 0:
                data.append({
                    "x": x_coords[j],
                    "y": y_coords[i],
                    "hits": grid_merged[i, j]
                })

    df = pd.DataFrame(data)

    # Saves the grid in a CSV file.
    df.to_csv(csv_path, index=False)
