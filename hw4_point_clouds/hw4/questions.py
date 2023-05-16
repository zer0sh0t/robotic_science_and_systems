from typing import Tuple
from scipy.spatial import cKDTree
import numpy as np
import utils


def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    center = np.mean(P, axis=0)
    cov = np.cov(P.T)
    lambdas, vecs = np.linalg.eig(cov)
    min_idx = np.argmin(lambdas)
    normal = vecs[:, min_idx]
    return (normal, center)


def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    n_iters = 10000
    epsilon = 0.0001
    min_n_inliers = float("-inf")
    best_normal = None
    best_center = None

    for _ in range(n_iters):
        ids = np.random.choice(len(P), 3, replace=False)
        p0, p1, p2 = P[ids]

        vec1 = p1 - p0
        vec2 = p2 - p0
        normal = np.cross(vec1, vec2)
        normal /= np.linalg.norm(normal)

        center = (p0 + p1 + p2) / 3
        dists = np.abs(np.dot(P - center, normal))

        inliers = (dists >= -1 * epsilon) & (dists <= epsilon)
        n_inliers = np.sum(inliers)

        if n_inliers > min_n_inliers:
            min_n_inliers = n_inliers
            best_normal = normal
            best_center = center

    return (best_normal, best_center)
    

def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''
    n_iters = 1000
    epsilon = 0.01
    min_n_inliers = float("-inf")
    best_c = None
    best_r = None

    for _ in range(n_iters):
        p_idx = np.random.randint(0, P.shape[0])
        r = np.random.uniform(0.05, 0.11)
        p = P[p_idx]
        n = N[p_idx]

        c = p + r * n # center = points + radius * normal
        dists = np.linalg.norm(P - c, axis=1)
        inliers = (dists >= (r - epsilon)) & (dists <= (r + epsilon))
        n_inliers = np.sum(inliers)

        if n_inliers > min_n_inliers:
            min_n_inliers = n_inliers
            best_c = c
            best_r = r

    return best_c, best_r


def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''
    n_iters = 5000
    epsilon = 0.001
    min_n_inliers = float("-inf")
    best_c = None
    best_a = None
    best_r = None

    for _ in range(n_iters):
        p_ids = np.random.choice(len(P), 2, replace=False)
        r = np.random.uniform(0.05, 0.1)
        p0, p1 = P[p_ids]
        n0, n1 = N[p_ids]

        a = np.cross(n0, n1)
        a /= np.linalg.norm(a)

        if np.random.uniform(0, 1) > 0.5:
            c = p0 + r * n0
        else:
            c = p1 + r * n1
        
        proj_mat = np.eye(3) - np.outer(a, a)
        P_ = P @ proj_mat
        c_ = c @ proj_mat
        
        dists = np.linalg.norm(P_ - c_, axis=1)
        inliers = (dists >= (r - epsilon)) & (dists <= (r + epsilon))
        n_inliers = np.sum(inliers)

        if n_inliers > min_n_inliers:
            min_n_inliers = n_inliers
            best_c = c_
            best_a = a
            best_r = r

    return best_c, best_a, best_r


def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
    
    M_mean = np.mean(M, axis=0)
    D_mean = np.mean(D, axis=0)
    M_ = M - M_mean
    D_ = D - D_mean

    W = np.dot(M_.T, D_)
    U, sigma, Vt = np.linalg.svd(W)
    R = Vt.T.dot(U.T)
    t = D_mean - R.dot(M_mean)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''
    
    T = np.eye(4)
    thresh = 1e-6
    n_iters = 50

    for _ in range(n_iters):
        idxs = find_closest_pc(M, D)

        T_i = q4_a(M[idxs], D)
        T = T_i.dot(T)
        M = apply_transform(M, T_i)
        
        norm = np.linalg.norm(T - np.eye(4))
        if norm < thresh:
            break
    
    return T


def find_closest_pc(M, D):
    idxs = []
    for d in D:
        dists = np.linalg.norm(M - d, axis=1)
        i = np.argmin(dists)
        idxs.append(i)
    return idxs


def apply_transform(P, T):
    P = T[:3, :3].dot(P.T).T + T[:3, 3]
    return P
