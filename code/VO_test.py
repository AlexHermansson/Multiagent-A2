import numpy as np
import matplotlib.pyplot as plt
from shapely import geometry

def create_VO(p):
    """Creating the velocity obstacle"""

    tau = 2
    v_max = 1.1
    r_A = 0.5; r_B = 0.5
    center = p / tau
    r = (r_A + r_B)/tau

    d = np.linalg.norm(center)
    theta = np.arccos(r / d)
    phi = np.arctan2(center[1], center[0])
    A_x = r * np.cos(np.pi + theta + phi)
    A_y = r * np.sin(np.pi + theta + phi)
    A = center + np.array([A_x, A_y])
    B_x = r * np.cos(np.pi - theta + phi)
    B_y = r * np.sin(np.pi - theta + phi)
    B = center + np.array([B_x, B_y])
    C = B + B / np.linalg.norm(B) * 3 * v_max # 2 v_max is as large as it gets, but we want a margin. See drawing.
    D = A + A / np.linalg.norm(A) * 3 * v_max

    return center, r, A, B, C, D # C and D are the furthest points in the polygon

def compute_u(p, v):
    center, r, A, B, C, D = create_VO(p)

    v_opt_rel = v

    polygon = geometry.Polygon([A, center,B, C, D])
    polygon_contains=geometry.Polygon([A,center, B, C, D]).contains(geometry.Point(v_opt_rel))
    circle_contains=geometry.Point(center).buffer(r).contains(geometry.Point(v_opt_rel))
    if polygon_contains:
        x_1 = C
        x_2 = D
        u_1 = np.dot(x_1, v_opt_rel)/(np.dot(x_1, x_1))*x_1 - v_opt_rel
        u_2 = np.dot(x_2, v_opt_rel)/(np.dot(x_2, x_2))*x_2 - v_opt_rel

        return (u_1, u_1/np.linalg.norm(u_1)) if np.linalg.norm(u_1) < np.linalg.norm(u_2) else (u_2, u_2/np.linalg.norm(u_2))

    elif circle_contains:
        a = (v_opt_rel - center)*r/np.linalg.norm( v_opt_rel- center)
        u= a-(v_opt_rel - center)
        return (u, u/np.linalg.norm(u))

    else:
        '''d1=geometry.LineString([A, D]).distance(geometry.Point(v_opt_rel))
        d2=geometry.Point(center).buffer(r).distance(geometry.Point(v_opt_rel))
        d3= geometry.LineString([B, C]).distance(geometry.Point(v_opt_rel))
        if d2 < d1 and d2<d3:
            u=(center-v_opt_rel)/np.linalg.norm(center-v_opt_rel)*d2
            return (u, -u/np.linalg.norm(u))
        elif d1 < d3:
            x_1 = D
            u_1 = np.dot(x_1, v_opt_rel) / (np.dot(x_1, x_1)) * x_1 - v_opt_rel
            return (u_1, -u_1 / np.linalg.norm(u_1))
        else:
            x_3=C
            u_3 = np.dot(x_3, v_opt_rel) / (np.dot(x_3, x_3)) * x_3 - v_opt_rel
            return (u_3, -u_3 / np.linalg.norm(u_3))'''
        d1 = geometry.LineString([A, D]).distance(geometry.Point(v_opt_rel))
        d2 = geometry.Point(center).buffer(r).distance(geometry.Point(v_opt_rel))
        d3 = geometry.LineString([B, C]).distance(geometry.Point(v_opt_rel))
        if d2 < d1 and d2 < d3:
            u = (center - v_opt_rel) / np.linalg.norm(center - v_opt_rel) * d2
            return (u, -u / np.linalg.norm(u))
        # todo: recheck this part maybe
        else:
            pol_ext = geometry.LinearRing(polygon.exterior.coords)
            d = pol_ext.project(geometry.Point(v_opt_rel))
            p = pol_ext.interpolate(d)
            closest_point_coords = np.array(list(p.coords)[0])
            u = closest_point_coords - v_opt_rel
            return (u, -u / np.linalg.norm(u))


p = np.array((1,1.2))
center, r, A, B, C, D = create_VO(p)

# circle
t = np.linspace(0, 2*np.pi)
x = center[0] + r*np.cos(t)
y = center[1] + r*np.sin(t)
plt.plot(x, y)

# polygon
x_p = np.array([A[0], B[0], C[0], D[0]])
y_p = np.array([A[1], B[1], C[1], D[1]])
plt.scatter(A[0], A[1],c='R')
plt.scatter(B[0], B[1],c='G')
plt.scatter(C[0], C[1],c='b')
plt.scatter(D[0], D[1],c='k')
#plt.plot(x_p, y_p)

plt.axis([-4, 5, -4, 4])


v = np.array([0.5, 0.5])

plt.scatter(v[0], v[1], marker='*')

u_ = compute_u(p, v)

if u_:
    u = u_[0]
    n = u_[1]
    plt.arrow(v[0], v[1], u[0], u[1], color='r', head_width=0.05, head_length=0.1)
    plt.arrow(v[0]+u[0], v[1]+u[1], n[0], n[1], head_width=0.05, head_length=0.1)
    print(u)
else:
    print('u is None')

plt.scatter(0, 0, marker='+')
plt.show()


