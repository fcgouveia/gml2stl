import networkx as nx
import open3d as o3d
import numpy as np

def create_stl(graph, filename):
    spheres = o3d.geometry.TriangleMesh()

    # Create spheres for nodes
    for node, data in graph.nodes(data=True):
        x, y, z = data['x'] * 50, data['y'] * 50, data['z'] * 10
        radius = data['degree'] * 0.2
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
        sphere.translate([x, y, z])
        spheres += sphere

    # Create cylinders for edges
    for edge in graph.edges(data=True):
        source, target, data = edge
        source_data, target_data = graph.nodes[source], graph.nodes[target]

        # Calculate positions
        source_pos = [source_data['x'] * 50, source_data['y'] * 50, source_data['z'] * 10]
        target_pos = [target_data['x'] * 50, target_data['y'] * 50, target_data['z'] * 10]

        # Calculate distance using NumPy arrays
        distance = np.array(target_pos) - np.array(source_pos)
        height = np.linalg.norm(distance)  # Calculate magnitude as scalar height

        # Calculate rotation matrix directly
        if np.linalg.norm(distance) > 0:
            direction = distance / np.linalg.norm(distance)
            angle = np.arccos(np.dot([0, 0, 1], direction))
            axis = np.cross([0, 0, 1], direction)
            axis /= np.linalg.norm(axis)

            # Create rotation matrix
            c = np.cos(angle)
            s = np.sin(angle)
            t = 1 - c
            x, y, z = axis
            rotation_matrix = np.array([
                [t * x**2 + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y**2 + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z**2 + c]
            ])

            # Create cylinder with rotation
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=data['value']**0.2*0.6, height=height)
            # Rotate the cylinder
            cylinder.rotate(rotation_matrix, center=(0, 0, 0))

            # Translate the cylinder to connect the edges
            translation_vector = np.array(source_pos) + 0.5 * distance
            cylinder.translate(translation_vector)

            spheres += cylinder

    # Compute normals for the final mesh
    spheres.compute_vertex_normals()

    # Save the final mesh as STL
    o3d.io.write_triangle_mesh(filename, spheres)

def gml_to_stl(input_file, output_file):
    graph = nx.read_gml(input_file)
    create_stl(graph, output_file)

if __name__ == "__main__":
    gml_file = "2Dgraph.gml"  # Replace with your GML file path
    stl_file = "3Dgraph.stl"  # Replace with desired output STL file path

    gml_to_stl(gml_file, stl_file)
