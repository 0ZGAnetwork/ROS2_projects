import matplotlib.pyplot as plt
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

x0.set_estimate(g2o.SE2(0, 0, 0))
x0.set_fixed(True)
x1.set_estimate(g2o.SE2(2, 0, 0))
x2.set_estimate(g2o.SE2(4, 0, 0))
f1.set_estimate(g2o.SE2(1, 1, 0))
f2.set_estimate(g2o.SE2(3, -1, 0))

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

plt.figure()
for v in [x0, x1, x2]:
    pose = v.estimate()
    t = pose.translation()
    plt.plot(t[0], t[1], 'bo')
    plt.text(t[0], t[1], f"X{v.id()}")
for v in [f1, f2]:
    pose = v.estimate()
    t = pose.translation()
    plt.plot(t[0], t[1], 'rs')
    plt.text(t[0], t[1], f"F{v.id()-2}")
for e in optimizer.edges():
    v1 = e.vertex(0).estimate().translation()
    v2 = e.vertex(1).estimate().translation()
    plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-')
plt.axis('equal')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('2D Pose Graph Optimization with g2o')
plt.grid()
plt.show()