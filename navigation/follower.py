class turn_counter:
    """
    Inputs: values from sensors, junction lists
    Outputs: junction value
    """

    def __init__(self):
        self.left_junc_count = 0
        self.right_junc_count = 0
        self.cross_junc_count = 0

    def direction(self, vals, left_juncs, right_juncs, cross_juncs):
        if vals == [1, 1, 1, 0]:
            self.left_junc_count += 1
            junction = left_juncs[self.left_junc_count]   # left junction
        elif vals == [0, 1, 1, 1]:
            self.right_junc_count += 1
            junction = right_juncs[self.right_junc_count]   # right junction
        elif vals == [1, 1, 1, 1]:
            self.cross_junc_count += 1
            junction = cross_juncs[self.cross_junc_count] # cross junction
        else:
            junction = 0
        
        return junction