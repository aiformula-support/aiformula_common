import numpy as np
from scipy.interpolate import interp1d
from scipy.stats import multivariate_normal


class RoadEstimation:
    def __init__(self, road_sigma, road_risk_table):
        Weight_u = [-0.1, 0., 0.41, 0.69, 0.83, 1., 1.1]
        Weight_y1 = [1., 1., 0., 0., 0., 0., 0.]
        Weight_y2 = [0., 0., 1., 0., 0., 0., 0.]
        Weight_y3 = [0., 0., 0., 1., 0., 0., 0.]
        Weight_y4 = [0., 0., 0., 0., 1., 0., 0.]
        Weight_y5 = [0., 0., 0., 0., 0., 1., 1.]
        self.Weight1_function = interp1d(Weight_u, Weight_y1)
        self.Weight2_function = interp1d(Weight_u, Weight_y2)
        self.Weight3_function = interp1d(Weight_u, Weight_y3)
        self.Weight4_function = interp1d(Weight_u, Weight_y4)
        self.Weight5_function = interp1d(Weight_u, Weight_y5)
        self.max_point_length = 50
        identification_gain = 10
        self.identification_gain_matrix = np.zeros((3, 3))
        np.fill_diagonal(
            self.identification_gain_matrix, identification_gain)

        self.forget_vector = np.array([0.99995, 0.99995, 0.98])
        self.mem_lim = 0.3
        self.theta_base = [0., 0., 0.]
        self.risk_gain = 10
        sigma = road_sigma  # sigma is distance between seek_point and road_bound
        self.sigma_max = max(sigma)
        self.sigma_min = min(sigma)

        ## Roadbound Risk Potential Function ##
        left_road_risk_table = road_risk_table
        self.left_road_risk = interp1d(sigma, left_road_risk_table)
        right_road_risk_table = list(reversed(road_risk_table))
        self.right_road_risk = interp1d(sigma, right_road_risk_table)

    def line_identification(self, xys):
        xs = np.ravel(xys[:, :1].T)
        norm_ofs = min(xs)  # offset
        norm_den = max(xs) - norm_ofs  # max limit
        index = np.arange(0, len(xs))
        num_xs = min(len(xs), self.max_point_length)
        index = np.random.permutation(len(xs))
        xs = xs[index]
        xs = (xs - norm_ofs) / norm_den  # xs are normalize 1xN

        ys = np.ravel(xys[:, 1:2].T)  # 1xN
        ys = ys[index]

        xxs = xs * xs
        ones = np.array([1] * len(xs))
        zetas = np.vstack([xxs, xs, ones]).T  # memory by Nx(1x3)
        w1s = self.Weight1_function(xs)
        w2s = self.Weight2_function(xs)
        w3s = self.Weight3_function(xs)
        w4s = self.Weight4_function(xs)
        w5s = self.Weight5_function(xs)

        thetas = np.zeros((5, 3))  # memory by 5x(1x3)
        dthetas = np.zeros((5, 3))  # memory by 5x(1x3)

        for zeta, w1, w2, w3, w4, w5, y in zip(zetas, w1s, w2s, w3s, w4s, w5s, ys):
            # preparation any parameters
            # p: 3x3, zeta: 3x1 = 3x1
            adaptive_gain_num = np.dot(
                self.identification_gain_matrix, zeta.reshape(3, 1))
            # zeta: 1x3, P: 3x3 = 1x3
            zp = np.dot(zeta.reshape(1, 3), self.identification_gain_matrix)
            # zp: 1x3, zeta: 3x1 = zpz: 1x1
            adaptive_gain_den = 1 + np.dot(zp, zeta.reshape(3, 1))
            adaptive_gain = adaptive_gain_num / adaptive_gain_den  # num: 3x1, den = 1x1

            # Estimate & Error
            # Theta.T: 1x3, zeta: 3x1 = scalar
            y_hat1 = np.dot(thetas[0], zeta.T) * w1
            y_hat2 = np.dot(thetas[1], zeta.T) * w2
            y_hat3 = np.dot(thetas[2], zeta.T) * w3
            y_hat4 = np.dot(thetas[3], zeta.T) * w4
            y_hat5 = np.dot(thetas[4], zeta.T) * w5
            y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5
            y_error = y - (y_hat)  # scalar
            y_error = np.clip(y_error, -1, 1)
            weight_y_error1 = (y_error * w1) * \
                adaptive_gain  # adaptive_gain: 3x1
            weight_y_error2 = (y_error * w2) * adaptive_gain
            weight_y_error3 = (y_error * w3) * adaptive_gain
            weight_y_error4 = (y_error * w4) * adaptive_gain
            weight_y_error5 = (y_error * w5) * adaptive_gain

            # parameter identifier
            thetas[0], dthetas[0] = self.f_identifier(
                np.ravel(weight_y_error1), thetas[0], dthetas[0])
            thetas[1], dthetas[1] = self.f_identifier(
                np.ravel(weight_y_error2), thetas[1], dthetas[1])
            thetas[2], dthetas[2] = self.f_identifier(
                np.ravel(weight_y_error3), thetas[2], dthetas[2])
            thetas[3], dthetas[3] = self.f_identifier(
                np.ravel(weight_y_error4), thetas[3], dthetas[3])
            thetas[4], dthetas[4] = self.f_identifier(
                np.ravel(weight_y_error5), thetas[4], dthetas[4])

        return norm_den, norm_ofs, thetas

    def f_identifier(self, ddtheta, dtheta, dtheta_mem):
        # ddtheta: 1x3, dtheta_mem: 1x3 ... add 1x3 + 1x3
        dtheta_mem_ret = ddtheta + (dtheta_mem * self.forget_vector)
        dtheta_mem_ret[0] = max(dtheta_mem_ret[0], -self.mem_lim)
        dtheta_mem_ret[0] = min(dtheta_mem_ret[0], self.mem_lim)
        dtheta_ret = dtheta_mem_ret + np.array(self.theta_base)  # 1x3

        return dtheta_ret, dtheta_mem_ret  # for 1x3

    def estimate_yhat(self, x_u, norm_den, norm_ofs, thetas):  # thetas is memory by 5x(1x3)
        x = (x_u - norm_ofs) / norm_den  # xs are normalize
        xx = x * x
        zeta = np.array([xx, x, 1.]).T  # memory by Nx(1x3)
        w1 = self.Weight1_function(x)
        w2 = self.Weight2_function(x)
        w3 = self.Weight3_function(x)
        w4 = self.Weight4_function(x)
        w5 = self.Weight5_function(x)

        # Theta.T: 1x3, zeta: 3x1 = scalar
        y_hat1 = np.dot(thetas[0], zeta.T) * w1
        y_hat2 = np.dot(thetas[1], zeta.T) * w2
        y_hat3 = np.dot(thetas[2], zeta.T) * w3
        y_hat4 = np.dot(thetas[3], zeta.T) * w4
        y_hat5 = np.dot(thetas[4], zeta.T) * w5
        y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5

        return y_hat

    def calculation_road_risk(self, seek_ps, norm_den, norm_ofs, thetas):
        # seeks_ps: 3 x (2x5)
        risks = []
        for seek_p in seek_ps:
            risk = [0, 0, 0, 0, 0]
            y_hat = 0
            seek_x = seek_p[0][2]  # center
            seek_ys = [p for p in seek_p[1]]
            if (seek_x > norm_ofs) and (seek_x < (norm_ofs + norm_den)):
                y_hat = self.estimate_yhat(seek_x, norm_den, norm_ofs, thetas)

            if y_hat:
                for i in range(len(seek_ys)):
                    # distance between seek_point and road_bound
                    sigma = seek_ys[i] - y_hat
                    # sigma range check
                    sigma = np.clip(sigma, self.sigma_min, self.sigma_max)
                    if sigma > 0:
                        risk[i] = self.risk_gain * self.left_road_risk(sigma)
                    elif sigma < 0:
                        risk[i] = self.risk_gain * self.right_road_risk(sigma)
            risks.append(risk)
        return risks, y_hat  # list [5x1] x 3

    def benefit_path(self, seek_ps, y_hat_l, y_hat_r):
        benefits = []
        for seek_p in seek_ps:
            benefit = [0, 0, 0, 0, 0]
            y_hat_c = (y_hat_l+y_hat_r)/2
            scale = 1.0
            covariance = [1.0]
            seek_ys = [p for p in seek_p[1]]

            for i in range(len(seek_ys)):
                benefit[i] = scale * \
                    multivariate_normal.pdf(
                        seek_ys[i], mean=y_hat_c, cov=covariance)
            benefits.append(benefit)

        return benefits  # list [5x1] x 3
