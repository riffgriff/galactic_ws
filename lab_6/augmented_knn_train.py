#!/usr/bin/env python3
import cv2
import argparse
import csv
import math
import pickle
import numpy as np
import random

def random_crop(img, min_crop_ratio=0.8, max_crop_ratio=0.95):
    h, w = img.shape[:2]
    crop_ratio = random.uniform(min_crop_ratio, max_crop_ratio)
    crop_h = max(1, int(h * crop_ratio))
    crop_w = max(1, int(w * crop_ratio))
    y = random.randint(0, h - crop_h)
    x = random.randint(0, w - crop_w)
    cropped = img[y:y + crop_h, x:x + crop_w]
    return cv2.resize(cropped, (w, h))

def random_rotation(img, min_angle=-20, max_angle=20):
    h, w = img.shape[:2]
    angle = random.uniform(min_angle, max_angle)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

def random_scaling(img, min_scale=0.85, max_scale=1.15):
    h, w = img.shape[:2]
    scale = random.uniform(min_scale, max_scale)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), 0, scale)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

def color_jitter(img, brightness_delta=20, contrast_min=0.85, contrast_max=1.15, sat_min=0.85, sat_max=1.15):
    jittered = img.astype(np.float32)
    contrast = random.uniform(contrast_min, contrast_max)
    brightness = random.uniform(-brightness_delta, brightness_delta)
    jittered = np.clip(jittered * contrast + brightness, 0, 255).astype(np.uint8)

    hsv = cv2.cvtColor(jittered, cv2.COLOR_BGR2HSV).astype(np.float32)
    sat_scale = random.uniform(sat_min, sat_max)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat_scale, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

def augment_image(img):
    return [
        img,
        random_crop(img),
        random_rotation(img),
        cv2.flip(img, 1),
        cv2.flip(img, 0),
        random_scaling(img),
        color_jitter(img),
    ]

def check_split_value_range(val):
    try:
        float_val = float(val)
        if float_val < 0 or float_val > 1:
            raise argparse.ArgumentTypeError("Received data split ratio of %s which is an invalid value. The input ratio must be in range [0, 1]!" % float_val)
        return float_val
    except ValueError:
        raise argparse.ArgumentTypeError(f"Received '{val}' which is not a valid float!")

def check_k_value(val):
    try:
        int_val = int(val)
        if float(val) != int_val:
            raise argparse.ArgumentTypeError(f"Received '{val}' which is a float not an integer. The KNN value input must be an integer!")
        if int_val % 2 == 0 or int_val < 1:
            raise argparse.ArgumentTypeError(f"Received '{val}' which not a positive, odd integer. The KNN value input must be a postive, odd integer!")
        return int_val
    except ValueError:
        raise argparse.ArgumentTypeError(f"Received '{val}' which is not a valid integer!")

def load_and_split_data(data_path, split_ratio):
    with open(data_path + 'labels.txt', 'r') as f:
        reader = csv.reader(f)
        lines = list(reader)

    random.shuffle(lines)
    train_lines = lines[:math.floor(len(lines) * split_ratio)][:]
    test_lines = lines[math.floor(len(lines) * split_ratio):][:]

    return train_lines, test_lines

def train_model(data_path, train_lines, image_type, model_filename, save_model):
    train_images = []
    train_labels = []

    for i in range(len(train_lines)):
        image = cv2.imread(data_path + train_lines[i][0] + image_type)
        if image is None:
            continue
        label = np.int32(train_lines[i][1])
        augmented_images = augment_image(image)
        train_images.extend(augmented_images)
        train_labels.extend([label] * len(augmented_images))

    train = np.array([np.array(cv2.resize(img, (25, 33))) for img in train_images])

    train_data = train.flatten().reshape(len(train_images), 33 * 25 * 3)
    train_data = train_data.astype(np.float32)

    train_labels = np.array(train_labels, dtype=np.int32)

    print(f"Training with {len(train_images)} images after augmentation.")

    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    print("KNN model created!")

    if save_model:
        knn.save(model_filename + '.xml')
        print(f"KNN model saved to {model_filename}.xml")

    return knn

def test_model(data_path, test_lines, image_type, knn_model, knn_value, show_img):
    if(show_img):
        Title_images = 'Original Image'
        Title_resized = 'Image Resized'
        cv2.namedWindow(Title_images, cv2.WINDOW_AUTOSIZE)

    correct = 0.0
    confusion_matrix = np.zeros((6, 6))

    k = knn_value

    for i in range(len(test_lines)):
        original_img = cv2.imread(data_path + test_lines[i][0] + image_type)
        test_img = np.array(cv2.resize(cv2.imread(data_path + test_lines[i][0] + image_type), (25, 33)))
        if(show_img):
            cv2.imshow(Title_images, original_img)
            cv2.imshow(Title_resized, test_img)
            key = cv2.waitKey()
            if key == 27:
                break
        test_img = test_img.flatten().reshape(1, 33 * 25 * 3)
        test_img = test_img.astype(np.float32)

        test_label = np.int32(test_lines[i][1])

        ret, results, neighbours, dist = knn_model.findNearest(test_img, k)

        if test_label == ret:
            print(str(test_lines[i][0]) + " Correct, " + str(ret))
            correct += 1
            confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        else:
            confusion_matrix[test_label][np.int32(ret)] += 1

            print(str(test_lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
            print("\tneighbours: " + str(neighbours))
            print("\tdistances: " + str(dist))

    print("\n\nTotal accuracy: " + str(correct / len(test_lines)))
    print(confusion_matrix)

def main():
    parser = argparse.ArgumentParser(description="Augmented KNN Trainer and Tester for 7785 Lab 6")
    parser.add_argument("-p", "--data_path", type=str, required=True, help="Path to the valid dataset directory (must contain labels.txt and images)")
    parser.add_argument("-r", "--data_split_ratio", type=check_split_value_range, required=False, default=0.9, help="Ratio of the train, test split.")
    parser.add_argument("-k", "--knn-value", type=check_k_value, required=False, default=1, help="KNN value. Must be an odd integer greater than zero.")
    parser.add_argument("-i", "--image_type", type=str, required=False, default=".png", help="Extension of the image files (e.g. .png, .jpg)")
    parser.add_argument("-s", "--save_model_bool", action='store_true', required=False, help="Boolean flag to save the KNN model as an XML file for later use.")
    parser.add_argument("-n", "--model_filename", type=str, required=False, default="knn_k1_r09_aug", help="Filename of the saved KNN model.")
    parser.add_argument("-t", "--dont_test_model_bool", action='store_false', required=False, help="Boolean flag to not test the created KNN model on split testing set (training only).")
    parser.add_argument("-d", "--show_img", action='store_true', required=False, help="Boolean flag to show the tested images as they are classified.")

    args = parser.parse_args()

    train_lines, test_lines = load_and_split_data(args.data_path, args.data_split_ratio)
    knn_model = train_model(args.data_path, train_lines, args.image_type, args.model_filename, args.save_model_bool)
    if args.dont_test_model_bool:
        test_model(args.data_path, test_lines, args.image_type, knn_model, args.knn_value, args.show_img)

if __name__ == "__main__":
    main()
