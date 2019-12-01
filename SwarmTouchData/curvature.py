from scipy import optimize
import numpy as np
import matplotlib.pyplot as plt


class ComputeCurvature:
    def __init__(self):
        """ Initialize some variables """
        self.xc = 0  # X-coordinate of circle center
        self.yc = 0  # Y-coordinate of circle center
        self.r = 0   # Radius of the circle
        self.xx = np.array([])  # Data points
        self.yy = np.array([])  # Data points

    def calc_r(self, xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return np.sqrt((self.xx-xc)**2 + (self.yy-yc)**2)

    def f(self, c):
        """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        ri = self.calc_r(*c)
        return ri - ri.mean()

    def df(self, c):
        """ Jacobian of f_2b
        The axis corresponding to derivatives must be coherent with the col_deriv option of leastsq"""
        xc, yc = c
        df_dc = np.empty((len(c), x.size))

        ri = self.calc_r(xc, yc)
        df_dc[0] = (xc - x)/ri                   # dR/dxc
        df_dc[1] = (yc - y)/ri                   # dR/dyc
        df_dc = df_dc - df_dc.mean(axis=1)[:, np.newaxis]
        return df_dc

    def fit(self, xx, yy):
        self.xx = xx
        self.yy = yy
        center_estimate = np.array([np.mean(xx), np.mean(yy)])
        center = optimize.leastsq(self.f, center_estimate, Dfun=self.df, col_deriv=True)[0]

        self.xc, self.yc = center
        ri = self.calc_r(*center)
        self.r = ri.mean()

        return 1 / self.r  # Return the curvature


# Apply code for an example
# x = np.array([36, 36, 19, 18, 33, 26])
# y = np.array([14, 10, 28, 31, 18, 26])

x = np.array([-1.45340233, -1.6055418,  -1.68380376, -1.71244107, -1.72538625])
y = np.array([1.20695993, 1.20967332, 1.13966997, 1.03852629, 0.92710937])

def plotter(x,y):
    comp_curv = ComputeCurvature()
    curv = comp_curv.fit(x, y)

    # Plot the result
    theta_fit = np.linspace(-np.pi, np.pi, 180)
    x_fit = comp_curv.xc + comp_curv.r*np.cos(theta_fit)
    y_fit = comp_curv.yc + comp_curv.r*np.sin(theta_fit)
    plt.plot(x_fit, y_fit, 'k--', label='fit', lw=2)
    plt.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
    plt.xlabel('x')
    plt.ylabel('y')
    # plt.title('curvature = {:.3e}'.format(curv))
    # plt.show()
    return curv

# plotter(y,-x)
# plt.show()