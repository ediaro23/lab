             
import numpy as np

class Bezier:   
    def __init__(self, pointlist, t_min=0.0, t_max=1.0, mult_t=1.0):
        self.dim_ = pointlist[0].shape[0]
        self.T_min_ = t_min
        self.T_max_ = t_max
        self.mult_T_ = mult_t
        self.size_ = len(pointlist)- 1
        self.degree_ = self.size_ - 1
        self.control_points_ = pointlist    
        if (self.size_ < 1 or self.T_max_ <= self.T_min_):
            raise ValueError("Can't create Bezier curve; min bound is higher than max bound.")
            
    def __call__(self, t):
        if not (self.T_min_ <= t <= self.T_max_):
            raise ValueError("Can't evaluate Bezier curve, time t is out of range")        
        if self.size_ == 1:
            return self.mult_T_ * self.control_points_[0]
        else:
            return self.eval_horner(t)
        
    def eval_horner(self, t):
        u = (t - self.T_min_) / (self.T_max_ - self.T_min_)
        u_op, bc, tn = 1.0 - u, 1, 1
        tmp = self.control_points_[0] * u_op
        for i in range(1, self.degree_):
            tn *= u
            bc *= (self.degree_ - i + 1) / i
            tmp = (tmp + tn * bc * self.control_points_[i]) * u_op
        return (tmp + tn * u * self.control_points_[-1]) * self.mult_T_
                
    def derivative(self, order):
        if order == 0:
            return self

        derived_wp = []
        for point1, point2 in zip(self.control_points_[:-1], self.control_points_[1:]):
            derived_wp.append(self.degree_ * (point2 - point1))

        if not derived_wp:
            derived_wp.append([0] * self.dim_)

        deriv = Bezier(derived_wp, self.T_min_, self.T_max_, self.mult_T_ * (1.0 / (self.T_max_ - self.T_min_)))
        return deriv.derivative(order - 1)

    