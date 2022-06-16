
class HubConfig():
    def __init__(self):
        self.slider_rec_pos = (None,None, None)
        self.slider_send_pos = (None, None, None)

        # positions for the toolhead to collect from pigeon holes
        self.toolhead_collect_pos = []

        # positions for the toolhead to deposit from pigeon holes
        # this positions should be have a greater z value than the
        # corresponding values in collect_pos
        self.toolhead_deposit_pos = []