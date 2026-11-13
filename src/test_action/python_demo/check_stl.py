import trimesh
import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stl_path", required=True)
    args = parser.parse_args()
    return args

def show_stl_info(stl_path):
    mesh = trimesh.load(stl_path)

    # 获取 XYZ 三个方向的尺寸
    print("Bounding Box:")
    print("X:", mesh.extents[0])
    print("Y:", mesh.extents[1])
    print("Z:", mesh.extents[2])

    # 获取模型边界
    print("Bounds:")
    print(mesh.bounds)


if __name__ == "__main__":
    args = get_args()
    show_stl_info(args.stl_path)
    