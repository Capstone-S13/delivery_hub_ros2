
class HubConfig():
    def __init__(self):
        self.slider_rec_pos = (15.0, 0.0, 445.0)
        self.slider_send_pos = (362.0, 0.0, 90.0)

        # positions for the toolhead to collect from pigeon holes
        self.toolhead_collect_pos = [(15.0, 0.0, 680.0), (362.0, 0.0, 680.0), (15.0, 0.0, 1030.0), (362.0, 0.0, 1030.0)]

        # positions for the toolhead to deposit from pigeon holes
        # this positions should be have a greater z value than the
        # corresponding values in collect_pos
        self.toolhead_deposit_pos = [(15.0, 0.0, 585.0), (362.0, 0.0, 585.0), (15.0, 0.0, 935.0), (362.0, 0.0, 935.0)]
