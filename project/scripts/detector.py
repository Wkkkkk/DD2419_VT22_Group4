"""Baseline detector model.

Inspired by
You only look once: Unified, real-time object detection, Redmon, 2016.
"""
from matplotlib import image
import numpy as np
import torch
import torch.nn as nn
from torchvision import models
from torchvision import transforms
import utils
from PIL import Image, ImageFilter

# Data augmentation part
import matplotlib.pyplot as plt
import torchvision.transforms.functional as TF
from random import randint, randrange

NUM_CATEGORIES = 15

is_rocm_pytorch = True

class Detector(nn.Module):
    """Baseline module for object detection. One single network for both classification and detection
    MobilnetV2 is a pretrained network used only to extract features, those features are then used to train the
    network for detection and classification. Could have used two different networks for detection and classification
    but would have had to extract features twice. Went for a single network as it seemed at first it would be faster."""

    def __init__(self):
        """Create the module.

        Define all trainable layers.
        """
        super(Detector, self).__init__()

        self.features = models.mobilenet_v2(pretrained=True).features #USED ONLY FOR FEATURE EXTRACTION.
        # output of mobilenet_v2 will be 1280x15x20 for 480x640 input images

        self.head = nn.Conv2d(in_channels=1280, out_channels=5 +
                              NUM_CATEGORIES, kernel_size=1)
        # 1x1 Convolution to reduce channels to out_channels without changing H and W

        # 1280x15x20 -> 5x15x20, where each element 5 channel tuple corresponds to
        #   (rel_x_offset, rel_y_offset, rel_x_width, rel_y_height, confidence
        # Where rel_x_offset, rel_y_offset is relative offset from cell_center
        # Where rel_x_width, rel_y_width is relative to image size
        # Where confidence is predicted IOU * probability of object center in this cell
        self.out_cells_x = 20.0
        self.out_cells_y = 15.0
        self.img_height = 480.0
        self.img_width = 640.0

    def forward(self, inp):
        """Forward pass.

        Compute output of neural network from input.
        """
        features = self.features(inp)
        out = self.head(features)  # Linear (i.e., no) activation

        return out

    def decode_output(self, out, threshold=None, topk=100):
        """Convert output to list of bounding boxes and find the Bounding box with a confidence higher than a given "threshold", then look for the
        corresponding 15 channels for that bounding box and find the one with the highest probability, that will be the corresponding class (Traffic sign) that
        the network has predicted.

        Args:
            out (torch.tensor):
                The output of the network.
                Shape expected to be NxCxHxW with
                    N = batch size
                    C = channel size
                    H = image height
                    W = image width
            threshold (Optional[float]):
                The threshold above which a bounding box will be accepted.
                If None, the topk bounding boxes will be returned.
            topk (int):
                Number of returned bounding boxes if threshold is None.
        Returns:
            List[List[Dict]]
            List containing a list of detected bounding boxes in each image.
            Each dictionary contains the following keys:
                - "x": Top-left corner column
                - "y": Top-left corner row
                - "width": Width of bounding box in pixel
                - "height": Height of bounding box in pixel
                - "score": Confidence score of bounding box
                - "category": Category (not implemented yet!)
        """
        bbs = []
        out = out.cpu()
        # decode bounding boxes for each image
        for o in out:
            img_bbs = []

            # find cells with bounding box center
            if threshold is not None:
                bb_indices = torch.nonzero(o[4, :, :] >= threshold)
            else:
                _, flattened_indices = torch.topk(o[4, :, :].flatten(), topk)
                bb_indices = np.array(
                    np.unravel_index(flattened_indices.numpy(), o[4, :, :].shape)
                ).T

            # loop over all cells with bounding box center
            for bb_index in bb_indices:
                bb_coeffs = o[0:4, bb_index[0], bb_index[1]]

                # decode bounding box size and position
                width = self.img_width * abs(bb_coeffs[2].item())
                height = self.img_height * abs(bb_coeffs[3].item())
                y = (
                    self.img_height / self.out_cells_y * (bb_index[0] + bb_coeffs[1])
                    - height / 2.0
                ).item()
                x = (
                    self.img_width / self.out_cells_x * (bb_index[1] + bb_coeffs[0])
                    - width / 2.0
                ).item()

                # Find index of channel with highest probability for classification
                category_probability_ind = np.argmax(o[5:, bb_index[0], bb_index[1]])
                # Get confidence of bounding box with highest probability.
                confidence = o[4, bb_index[0], bb_index[1]]
                # print(category_probability_ind.item())

                img_bbs.append(
                    {
                        "width": width,
                        "height": height,
                        "x": x,
                        "y": y,
                        "category": category_probability_ind.item(),
                        # Confidence of bounding box
                        "score": o[4, bb_index[0], bb_index[1]].item(),
                    }
                )
            bbs.append(img_bbs)

        return bbs

    def input_transform(self, image, anns):
        """Prepare image and targets on loading. Normalize, data augmentation(blurring, colorjitter), set correct labels. ALOT of other data augmentation
        was tried, such as flipping, perspective transformation and translations but didnt yield better results.

        This function is called before an image is added to a batch. That is before we send it through the network.
        Must be passed as transforms function to dataset.

        Args:
            image (PIL.Image):
                The image loaded from the dataset.
            anns (List):
                List of annotations in COCO format.
        Returns:
            Tuple:
                - (torch.Tensor) The image.
                - (torch.Tensor) The network target containing the bounding box.
        """
        # Data augmentation on PIL image

        #fig, axs = plt.subplots(1, 2)
        #axs[0].imshow(image)
        #bbs = []
        #for ann in anns:
        #    bbs.append({
        #        "x": ann["bbox"][0],
        #        "y": ann["bbox"][1],
        #        "width": ann["bbox"][2],
        #        "height": ann["bbox"][3],
        #    })
        #utils.add_bounding_boxes(axs[0], bbs)
        
        #Add ColorJitter
        cj = transforms.ColorJitter(0.3, 0.3, 0.3, 0.3)
        image = cj(image)
        #if len(anns) > 0:
        #    axs[0].set_title(anns[0]["category_id"])

        #Add random gaussian blur
        image = image.filter(ImageFilter.GaussianBlur(radius = 0.1 *randrange(0, 15)))

         #Horizontal Flip 50% chance.
        flipped_categories = {2:3, 3:2, 4:5, 5:4, 11:12, 12:11} #For Labeling
        allowed_flips = [2,3,4,5,6,9,11,12]

        w, h = image.size
        
        x = randint(0,1)
        for ann in anns:
            if ann["category_id"] not in allowed_flips:
                #print("not allowed")
                x = 0
        if x == 1 and len(anns) != 0:
            image = TF.hflip(image)
            for ann in anns:
                ann["bbox"][0] = w - ann["bbox"][0] - ann["bbox"][2] - 1
                if ann["category_id"] in flipped_categories:
                    ann["category_id"] = flipped_categories[ann["category_id"]]


        #ra = transforms.RandomAffine(0, [0,0])
        #angle, translations, scale, shear = ra.get_params(
        #    ra.degrees, ra.translate, ra.scale, ra.shear, image.size)
        #image = TF.affine(image, angle, translations, scale, shear,
        #          resample=ra.resample, fillcolor=ra.fillcolor)

        #axs[1].imshow(image)
        #if len(anns) > 0:
        #    axs[1].set_title(anns[0]["category_id"])

        #Apply transform on annotations!
        #for ann in anns:
        #    ann["bbox"][0] += translations[0]
        #    ann["bbox"][1] += translations[1]

        #bbs = []
        #for ann in anns:
        #    bbs.append({
        #        "x": ann["bbox"][0],
        #        "y": ann["bbox"][1],
        #        "width": ann["bbox"][2],
        #        "height": ann["bbox"][3],
        #    })
        #utils.add_bounding_boxes(axs[1], bbs)
        #print("aaaaaaaaaaaAAAAAAAAAAAAAaa")
        #plt.show()


        # Convert PIL.Image to torch.Tensor
        image = transforms.ToTensor()(image)
        image = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )(image)

        # Convert bounding boxes to target format

        # First two channels contain relativ x and y offset of bounding box center
        # Channel 3 & 4 contain relative width and height, respectively
        # Last channel is 1 for cell with bounding box center and 0 without

        # If there is no bb, the first 4 channels will not influence the loss
        # -> can be any number (will be kept at 0)
        target = torch.zeros(5 + NUM_CATEGORIES, 15, 20)
        for ann in anns:
            x = ann["bbox"][0]
            y = ann["bbox"][1]
            width = ann["bbox"][2]
            height = ann["bbox"][3]
            category = ann["category_id"]

            x_center = x + width / 2.0
            y_center = y + height / 2.0
            x_center_rel = x_center / self.img_width * self.out_cells_x
            y_center_rel = y_center / self.img_height * self.out_cells_y
            x_ind = int(x_center_rel)
            y_ind = int(y_center_rel)
            x_cell_pos = x_center_rel - x_ind
            y_cell_pos = y_center_rel - y_ind
            rel_width = width / self.img_width
            rel_height = height / self.img_height

            # channels, rows (y cells), cols (x cells)
            target[4, y_ind, x_ind] = 1

            # bb size
            target[0, y_ind, x_ind] = x_cell_pos
            target[1, y_ind, x_ind] = y_cell_pos
            target[2, y_ind, x_ind] = rel_width
            target[3, y_ind, x_ind] = rel_height
            target[category + 5, y_ind, x_ind] = 1

        return image, target
    def input_transform2(self, image, anns):
        """Prepare image and targets on loading. Normalize, data augmentation(blurring, colorjitter), set correct labels. ALOT of other data augmentation
        was tried, such as flipping, perspective transformation and translations but didnt yield better results.

        This function is called before an image is added to a batch. That is before we send it through the network.
        Must be passed as transforms function to dataset.

        Args:
            image (PIL.Image):
                The image loaded from the dataset.
            anns (List):
                List of annotations in COCO format.
        Returns:
            Tuple:
                - (torch.Tensor) The image.
                - (torch.Tensor) The network target containing the bounding box.
        """

        # Convert PIL.Image to torch.Tensor
        image = transforms.ToTensor()(image)
        image = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )(image)

        # Convert bounding boxes to target format

        # First two channels contain relativ x and y offset of bounding box center
        # Channel 3 & 4 contain relative width and height, respectively
        # Last channel is 1 for cell with bounding box center and 0 without

        # If there is no bb, the first 4 channels will not influence the loss
        # -> can be any number (will be kept at 0)
        target = torch.zeros(5 + NUM_CATEGORIES, 15, 20)
        for ann in anns:
            x = ann["bbox"][0]
            y = ann["bbox"][1]
            width = ann["bbox"][2]
            height = ann["bbox"][3]
            category = ann["category_id"]

            x_center = x + width / 2.0
            y_center = y + height / 2.0
            x_center_rel = x_center / self.img_width * self.out_cells_x
            y_center_rel = y_center / self.img_height * self.out_cells_y
            x_ind = int(x_center_rel)
            y_ind = int(y_center_rel)
            x_cell_pos = x_center_rel - x_ind
            y_cell_pos = y_center_rel - y_ind
            rel_width = width / self.img_width
            rel_height = height / self.img_height

            # channels, rows (y cells), cols (x cells)
            target[4, y_ind, x_ind] = 1

            # bb size
            target[0, y_ind, x_ind] = x_cell_pos
            target[1, y_ind, x_ind] = y_cell_pos
            target[2, y_ind, x_ind] = rel_width
            target[3, y_ind, x_ind] = rel_height
            target[category + 5, y_ind, x_ind] = 1

        return image, target
