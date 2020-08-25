#!/usr/bin/env python3
import argparse
import os

import cv2 as cv
import PIL
import torch
import torchvision

class Net:
    def __init__(self, modelfile):
        self.classes = ["large_thick_gray_back", "large_thick_gray_front", "medium_thin_gray_back", "medium_thin_gray_front", "medium_thin_green_back", "medium_thin_green_front", "small_thin_blue_back", "small_thin_blue_front"]
        self.classes.sort()

        self.model = torchvision.models.resnet18()
        self.model.fc = torch.nn.Linear(self.model.fc.in_features, len(self.classes))
        self.model.eval()
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model.load_state_dict(torch.load(modelfile, map_location=self.device))

        resize = torchvision.transforms.Resize(224)
        to_tensor = torchvision.transforms.ToTensor()
        normalize = torchvision.transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        self.transform = torchvision.transforms.Compose([resize, to_tensor, normalize])

    def classify(self, frame):
        pil_image = PIL.Image.fromarray(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
        batch = torch.unsqueeze(self.transform(pil_image), 0)
        out = self.model(batch)
        max_value, max_idx = torch.max(out, 1)
        return self.classes[max_idx]

class ClampClassificator:
    def __init__(self, modelfile):
        self.net = Net(modelfile)


    def classify(self, frame):
        prediction = self.net.classify(frame)

        class_name = '_'.join(prediction.split('_')[:-1])

        if prediction.split('_')[-1] == "front":
            front_shown = True
        else:
            front_shown = False

        return class_name, front_shown
