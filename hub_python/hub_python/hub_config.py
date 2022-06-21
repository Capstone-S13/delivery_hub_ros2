
class HubConfig():
    def __init__(self):
        self.slider_rec_pos = (45.0,0.0, 445.0)
        self.slider_send_pos = (365.0, 0.0, 300.0)

        # positions for the toolhead to collect from pigeon holes
        self.toolhead_collect_pos = [
            # (45.0,0.0,670.0),
            (350.0,0.0,680.0),
            # (45.0,0.0,800.0),
            # (365.0,0.0,800.0)
        ]

        # positions for the toolhead to deposit from pigeon holes
        # this positions should be have a greater z value than the
        # corresponding values in collect_pos
        self.toolhead_deposit_pos = [
            # (45.0,0.0,615.0),
            (350.0,0.0,615.0),
            # (45.0,0.0,750.0),
            # (365.0,0.0,750.0)
        ]