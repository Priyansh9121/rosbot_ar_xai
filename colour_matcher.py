# color_matcher.py

import pandas as pd
import numpy as np
import os

class ColourMatcher:
    def __init__(self, csv_path="xkcd_colors.csv", use_tags=False):
        self.colors_df = pd.read_csv(csv_path)
        self.rgb_array = self.colors_df[['r', 'g', 'b']].to_numpy()
        self.color_names = self.colors_df['name'].tolist()
        self.use_tags = use_tags

    def match_color(self, rgb_input):
        """
        Match an input RGB (tuple of ints) to the closest named color.
        """
        rgb_input = np.array(rgb_input).reshape(1, -1)
        distances = np.linalg.norm(self.rgb_array - rgb_input, axis=1)
        idx = np.argmin(distances)
        color_name = self.color_names[idx]
        tags = self.get_tags(self.rgb_array[idx]) if self.use_tags else []
        return color_name, tags

    def get_tags(self, rgb):
        """
        Return semantic tags based on RGB values.
        """
        r, g, b = rgb
        tags = []

        # Warm/Cool
        if r > b:
            tags.append("warm")
        else:
            tags.append("cool")

        # Pastel/Vibrant
        if max(rgb) > 200 and min(rgb) > 150:
            tags.append("pastel")
        elif max(rgb) > 180:
            tags.append("vibrant")

        return tags
