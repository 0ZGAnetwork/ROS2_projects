import g2opy as g2o
import numpy as np

optimizer = g2o.SparseOptimizer()
solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))

x0 = g2o.VertexSE2(); x0.set_id(0); optimizer.add_vertex(x0)
x1 = g2o.VertexSE2(); x1.set_id(1); optimizer.add_vertex(x1)
x2 = g2o.VertexSE2(); x2.set_id(2); optimizer.add_vertex(x2)
f1 = g2o.VertexSE2(); f1.set_id(3); optimizer.add_vertex(f1)
f2 = g2o.VertexSE2(); f2.set_id(4); optimizer.add_vertex(f2)

def add_edge(v_from, v_to, dx, dy, dtheta):
    edge = g2o.EdgeSE2()
    edge.set_vertex(0, v_from)
    edge.set_vertex(1, v_to)
    edge.set_measurement(g2o.SE2(dx, dy, dtheta))
    edge.set_information(np.identity(3))
    optimizer.add_edge(edge)

add_edge(x0, x1, 2.1, 0, 0)      # c1
add_edge(x1, x2, 1.9, 0, 0)      # c2
add_edge(x0, f1, 0.5, 1.0, 0)    # c01
add_edge(x1, f1, -1.5, 1.0, 0)   # c11
add_edge(x1, f2, 1.0, -1.0, 0)   # c12
add_edge(x2, f2, -1.0, -1.0, 0)  # c22

optimizer.initialize_optimization()
optimizer.optimize(10)

def print_pose(v):
    pose = v.estimate()
    t = pose.translation() # [x, y]
    theta = pose.rotation().angle()  # radians
    print(f"x={t[0]:.3f}, y={t[1]:.3f}, theta={theta:.3f}")

for v in [x0, x1, x2, f1, f2]:
    print_pose(v)