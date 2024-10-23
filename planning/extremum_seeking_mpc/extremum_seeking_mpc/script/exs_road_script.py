import numpy as np
from scipy.interpolate import interp1d
from scipy.stats import multivariate_normal

class RoadEst:
    def __init__(self, road_sigma, road_risk_table_l, road_risk_table_r):
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
        self.identification_gain1 = np.array([10, 0, 0])
        self.identification_gain2 = np.array([0, 10, 0])
        self.identification_gain3 = np.array([0, 0, 10])
        self.identification_gain_matrix = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.forget_vector = np.array([0.99995, 0.99995, 0.98])
        self.mem_lim = 0.3
        self.theta_base = [0., 0., 0.]
        self.risk_gain = 10
        #sigma = [-2.4, -2.0, -1.6, -1.2, -0.8, -0.4, 0., 0.4, 0.8, 1.2, 1.6, 2.0, 2.4]
        sigma = road_sigma # sigma is distance between seek_point and road_bound
        self.sigma_max = max(sigma)
        self.sigma_min = min(sigma)
                
        ## Roadbound Risk Potential Function ##
        #road_risk_table_l = [0.,0.,0.,0.,0.,0.,0., 0., 0.2, 0.4, 0.6, 0.8, 1.0]
        road_risk_table_l = road_risk_table_l
        self.road_risk_l = interp1d(sigma,road_risk_table_l)
        #road_risk_table_r = [1.0, 0.8, 0.6, 0.4, 0.2, 0., 0.,0.,0.,0.,0.,0.,0.]
        road_risk_table_r = road_risk_table_r
        self.road_risk_r = interp1d(sigma,road_risk_table_r)
    
    def line_id_quintiple(self, xys):
        xs = np.ravel(xys[:, :1].T)
        norm_ofs = min(xs) # offset
        norm_den = max(xs) - norm_ofs # max limit
        index = np.arange(0, len(xs))
        num_xs = min(len(xs), self.max_point_length)
        index = np.sort(np.random.choice(index, num_xs, replace=False))
        xs = xs[index]
        xs = (xs - norm_ofs) / norm_den # xs are normalize 1xN

        ys = np.ravel(xys[:, 1:2].T) # 1xN
        ys = ys[index]
        y_first = ys[0]
        
        xxs = xs * xs
        ones = np.array([1] * len(xs))
        zetast = np.vstack([xxs, xs, ones]).T # memory by Nx(1x3)
        w1s = self.Weight1_function(xs)
        w2s = self.Weight2_function(xs)
        w3s = self.Weight3_function(xs)
        w4s = self.Weight4_function(xs)
        w5s = self.Weight5_function(xs)

        thetas = np.full((5,3), 0.)  # memory by 5x(1x3)
        dthetas = np.full((5,3), 0.) # memory by 5x(1x3)
        for i, (zetat, w1, w2, w3, w4, w5, y) in enumerate(zip(zetast, w1s, w2s, w3s, w4s, w5s, ys)):
            # preparation any parameters
            adaptive_gain_num = np.dot(self.identification_gain_matrix, zetat.reshape(3, 1)) # p: 3x3, zeta: 3x1 = 3x1
            zp = np.dot(zetat.reshape(1, 3), self.identification_gain_matrix) # zeta: 1x3, P: 3x3 = 1x3
            adaptive_gain_den = 1 + np.dot(zp, zetat.reshape(3, 1)) # zp: 1x3, zeta: 3x1 = zpz: 1x1
            adaptive_gain = adaptive_gain_num / adaptive_gain_den # num: 3x1, den = 1x1

            # Estimate & Error
            y_hat1 = np.dot(thetas[0], zetat.T) * w1 # Theta.T: 1x3, zeta: 3x1 = scalar
            y_hat2 = np.dot(thetas[1], zetat.T) * w2
            y_hat3 = np.dot(thetas[2], zetat.T) * w3
            y_hat4 = np.dot(thetas[3], zetat.T) * w4
            y_hat5 = np.dot(thetas[4], zetat.T) * w5
            y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5
            y_err = y - (y_hat) # scalar
            y_err = max(y_err, -1)
            y_err = min(y_err, 1)
            weight_y_err1 = (y_err * w1) * adaptive_gain # adaptive_gain: 3x1
            weight_y_err2 = (y_err * w2) * adaptive_gain
            weight_y_err3 = (y_err * w3) * adaptive_gain
            weight_y_err4 = (y_err * w4) * adaptive_gain
            weight_y_err5 = (y_err * w5) * adaptive_gain

            # parameter identifier
            thetas[0], dthetas[0] = self.f_Identifier(np.ravel(weight_y_err1), thetas[0], dthetas[0])
            thetas[1], dthetas[1] = self.f_Identifier(np.ravel(weight_y_err2), thetas[1], dthetas[1])
            thetas[2], dthetas[2] = self.f_Identifier(np.ravel(weight_y_err3), thetas[2], dthetas[2])
            thetas[3], dthetas[3] = self.f_Identifier(np.ravel(weight_y_err4), thetas[3], dthetas[3])
            thetas[4], dthetas[4] = self.f_Identifier(np.ravel(weight_y_err5), thetas[4], dthetas[4])

        return norm_den, norm_ofs, thetas, y_first

    def f_Identifier(self, ddtheta, dtheta, dtheta_mem):
        dtheta_mem_ret = ddtheta + (dtheta_mem * self.forget_vector) # ddtheta: 1x3, dtheta_mem: 1x3 ... add 1x3 + 1x3
        dtheta_mem_ret[0] = max(dtheta_mem_ret[0], -self.mem_lim)
        dtheta_mem_ret[0] = min(dtheta_mem_ret[0], self.mem_lim)
        #dtheta2 = -0.35053 * dtheta[2] + 1.4913
        #dtheta_ret = dtheta_mem_ret + np.array([self.theta_base[0], dtheta2, self.theta_base[2]]) # 1x3
        dtheta_ret = dtheta_mem_ret + np.array(self.theta_base) # 1x3

        return dtheta_ret, dtheta_mem_ret # for 1x3
    
    def EstimateYhat(self, x_u, norm_den, norm_ofs, thetas): # thetas is memory by 5x(1x3)
        x = (x_u - norm_ofs) / norm_den # xs are normalize
        xx = x * x
        zetat = np.array([xx, x, 1.]).T # memory by Nx(1x3)
        w1 = self.Weight1_function(x)
        w2 = self.Weight2_function(x)
        w3 = self.Weight3_function(x)
        w4 = self.Weight4_function(x)
        w5 = self.Weight5_function(x)

        y_hat1 = np.dot(thetas[0], zetat.T) * w1 # Theta.T: 1x3, zeta: 3x1 = scalar
        y_hat2 = np.dot(thetas[1], zetat.T) * w2
        y_hat3 = np.dot(thetas[2], zetat.T) * w3
        y_hat4 = np.dot(thetas[3], zetat.T) * w4
        y_hat5 = np.dot(thetas[4], zetat.T) * w5
        y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5

        return y_hat
    
    def CalculationRoadRisk(self, seek_ps, norm_den, norm_ofs, thetas):
        # seeks_ps: 3 x (2x5)
        risks = []
        for seek_p in seek_ps:
            risk = [0, 0, 0, 0, 0]
            y_hat = 0
            seek_x = seek_p[0][2] # center
            seek_ys = [p for p in seek_p[1]]
            if (seek_x > norm_ofs) and (seek_x < (norm_ofs + norm_den)):
                y_hat = self.EstimateYhat(seek_x, norm_den, norm_ofs, thetas)

            if y_hat:
                for i in range(len(seek_ys)):
                    sigma = seek_ys[i] - y_hat # distance between seek_point and road_bound
                    #sigma range check
                    sigma = max(sigma, self.sigma_min)
                    sigma = min(sigma, self.sigma_max)
                    if sigma > 0:
                        risk[i] = self.risk_gain * self.road_risk_l(sigma)
                    elif sigma < 0:
                        risk[i] = self.risk_gain * self.road_risk_r(sigma)
            risks.append(risk)
        #print(risks)
        return risks, y_hat # list [5x1] x 3
       
    def Benefit_path(self, seek_ps, y_hat_l, y_hat_r):
        benefits = []
        for seek_p in seek_ps:
            benefit = [0, 0, 0, 0, 0]
            y_hat_c  = (y_hat_l+y_hat_r)/2
            scale = 1.0
            covariance = [1.0]
            seek_ys = [p for p in seek_p[1]]

            for i in range(len(seek_ys)):
                benefit[i] = scale * multivariate_normal.pdf(seek_ys[i], mean= y_hat_c, cov=covariance)
            benefits.append(benefit)       

        return benefits  # list [5x1] x 3
