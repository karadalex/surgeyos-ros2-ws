import cv2
import numpy as np
import numpy as np


def centerOfMass(points):
    reshapedPoints = points.reshape(len(points), 2).T
    try:
        cmX = int(reshapedPoints[0].mean())
        cmY = int(reshapedPoints[1].mean())
        cm = np.array([cmX, cmY])
    except ValueError:
        cm = np.array([0, 0])
    return cm


# Covariance
def cov(x, y):
    try:
        xbar, ybar = int(x.mean()), int(y.mean())
    except ValueError:
        xbar, ybar = 0, 0
    return np.sum((x - xbar)*(y - ybar))/(len(x) - 1)


# Covariance matrix
def cov_mat(points):
    reshapedPoints = points.reshape(len(points), 2).T
    xi = reshapedPoints[0]
    yi = reshapedPoints[1]
    return np.array([[cov(xi, xi), cov(xi, yi)],
                     [cov(yi, xi), cov(yi, yi)]])


def floatUnitOrientationVectors(points):
    C = cov_mat(points)
    eigVal, eigVec = np.linalg.eig(C)
    return eigVec


def pixelizeFloatVector(vector):
    vector = vector*100
    return vector.astype(int)


def orientationVectors(points):
    eigVec = floatUnitOrientationVectors(points)
    v1 = pixelizeFloatVector(eigVec[0])
    v2 = pixelizeFloatVector(eigVec[1])
    return v1, v2


def points_inside_contour(contour, shape_hw):
    """
    Given a contour and the shape of the image, return the (x, y) coordinates of all points inside the contour.
    usage:
        pts, mask = points_inside_contour(cnt, (h, w))
    """
    h, w = shape_hw
    mask = np.zeros((h, w), dtype=np.uint8)

    # fill the contour
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

    # get (x, y) coordinates of non-zero pixels
    ys, xs = np.where(mask != 0)
    points = np.stack([xs, ys], axis=1)  # Nx2 as (x, y)
    return points, mask
