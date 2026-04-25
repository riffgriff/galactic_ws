#!/usr/bin/env python3
import cv2
import argparse
import csv
import math
import os
import pickle
import numpy as np
import random
import img_preprocessing

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

def load_and_split_data(data_paths, split_ratio, image_type):
    """
    Loads labels from all provided dataset folders, combines them, and splits once.

    Args:
        data_paths (list[str]): Paths to dataset folders.
        split_ratio (float): must be a float between 0 and 1. Split ratio will be used to split the data into training and testing sets. 
                             split_ratio of the data will be used for training and (1-split_ratio) will be used for testing. 
                             For example if split ratio was 0.7, 70% of the data will be used for training and the remaining 30% will be used for testing.
        image_type (str): Image extension to load.

    Returns:
        list of tuples for testing and training (image_path, true_label)
    """

    lines = []
    for data_path in data_paths:
        labels_file = os.path.join(data_path, 'labels.txt')
        with open(labels_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                image_name = row[0].strip()
                label = row[1].strip()
                image_path = os.path.join(data_path, image_name + image_type)
                lines.append((image_path, label))

    # Randomly choose train and test data from the combined dataset.
    random.shuffle(lines)
    train_lines = lines[:math.floor(len(lines)*split_ratio)][:]
    test_lines = lines[math.floor(len(lines)*split_ratio):][:]

    return train_lines, test_lines

def train_model(train_lines, model_filename, save_model):
    """
    Loads the images from the combined training set and uses them to create a KNN model.

    Args:
        train_lines (tuple): Tuple of the training data containing (image_path, true_label)

    Returns:
        knn (knn_model_object): The KNN model.
    """

    train_images = []
    train_labels = []

    for image_path, label in train_lines:
        img = cv2.imread(image_path)
        if img is None:
            continue
        x, y, w, h, _ = img_preprocessing.get_bounding_box(img)
        if img_preprocessing.is_reasonable_box(w,h):
            img = img[y:y+h,x:x+w] # crop image
        train_images.append(img)
        train_labels.append(np.int32(label))

    #This line reads in all images listed in the file in color, and resizes them to 25x33 pixels
    train = np.array([np.array(cv2.resize(img, (25,33))) for img in train_images])

    #Here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants), note the *3 is due to 3 channels of color.
    train_data = train.flatten().reshape(len(train_images), 33*25*3)
    train_data = train_data.astype(np.float32)

    #Read in training labels
    train_labels = np.array(train_labels, dtype=np.int32)

    if len(train_images) == 0:
        raise RuntimeError("No valid training images were found in the provided dataset folders.")

    ### Train classifier
    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    print("KNN model created!")

    if(save_model):
        # Save the trained model
        knn.save(model_filename + '.xml')

        print(f"KNN model saved to {model_filename}.xml")
    
    return knn

def test_model(test_lines, knn_model, knn_value, show_img):
    """
    Loads the images and tests the provided KNN model prediction with the dataset label.
    The images and labels must be in the given directoy.

    Args:
        data_path (str): Path to the dataset.
        test_lines (tuple): Tuple of the training data containing (image_number, true_label)
        image_type (str): Image extension to load (e.g. .png, .jpg, .jpeg)
        knn_model (model object): The knn model
        knn_value (int): The number of KNN neighbors to consider when classifying
        show_img: A boolean whether to show images as they are processed or not

    Returns:
        knn (knn_model_object): The KNN model.
    """

    if(show_img):
        Title_images = 'Original Image'
        Title_resized = 'Image Resized'
        cv2.namedWindow( Title_images, cv2.WINDOW_AUTOSIZE )

    correct = 0.0
    confusion_matrix = np.zeros((6,6))

    k = knn_value

    processed = 0
    correct = 0.0
    confusion_matrix = np.zeros((6,6))

    for image_path, label in test_lines:

        original_img = cv2.imread(image_path)
        if original_img is None:
            continue
        x, y, w, h, _ = img_preprocessing.get_bounding_box(original_img)
        if img_preprocessing.is_reasonable_box(w,h):
            img = original_img[y:y+h,x:x+w] # crop image
        else:
            img = original_img

        test_img = np.array(cv2.resize(img,(25,33)))
        if(show_img):
            cv2.imshow(Title_images, original_img)
            cv2.imshow(Title_resized, test_img)
            key = cv2.waitKey()
            if key==27:    # Esc key to stop
                break
        test_img = test_img.flatten().reshape(1, 33*25*3)
        test_img = test_img.astype(np.float32)

        test_label = np.int32(label)
        processed += 1

        ret, results, neighbours, dist = knn_model.findNearest(test_img, k)

        if test_label == ret:
            print(str(image_path) + " Correct, " + str(ret))
            correct += 1
            confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        else:
            confusion_matrix[test_label][np.int32(ret)] += 1
            
            print(str(image_path) + " Wrong, " + str(test_label) + " classified as " + str(ret))
            print("\tneighbours: " + str(neighbours))
            print("\tdistances: " + str(dist))



    print("\n\nTotal accuracy: " + str(correct/processed if processed else 0.0))
    print(confusion_matrix)

def main():
    parser = argparse.ArgumentParser(description="Example Model Trainer and Tester with Basic KNN for 7785 Lab 6!")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_dataset_paths = [
        os.path.join(base_dir, "2025F_imgs"),
        os.path.join(base_dir, "2025F_Gimgs"),
        os.path.join(base_dir, "2026S_imgs"),
    ]
    parser.add_argument("-p","--data_path", type=str, nargs='+', required=False, default=default_dataset_paths, help="Paths to the valid dataset directories (each must contain labels.txt and images)")
    parser.add_argument("-r","--data_split_ratio", type=check_split_value_range, required=False, default=0.5, help="Ratio of the train, test split. Must be a float between 0 and 1. The number entered is the percentage of data used for training, the remaining is used for testing!")
    parser.add_argument("-k","--knn-value", type=check_k_value, required=False, default=3, help="KNN value. Must be an odd integer greater than zero.")
    parser.add_argument("-i","--image_type", type=str, required=False, default=".png", help="Extension of the image files (e.g. .png, .jpg)")
    parser.add_argument("-s","--save_model_bool", action='store_true', required=False, help="Boolean flag to save the KNN model as an XML file for later use.")
    parser.add_argument("-n","--model_filename", type=str, required=False, default="knn_model", help="Filename of the saved KNN model.")
    parser.add_argument("-t","--dont_test_model_bool", action='store_false', required=False, help="Boolean flag to not test the created KNN model on split testing set (training only).")
    parser.add_argument("-d","--show_img", action='store_true', required=False, help="Boolean flag to show the tested images as they are classified.")


    args = parser.parse_args()

    #Path to dataset directory from command line argument.
    dataset_paths = args.data_path

    #Ratio of datasplit from command line argument.
    data_split_ratio = args.data_split_ratio

    #Image type from command line argument.
    image_type = args.image_type

    #Boolean if true will save the KNN model as a XML file from command line argument.
    save_model_bool = args.save_model_bool

    #Filename for the saved KNN model from command line argument.
    model_filename = args.model_filename

    #Boolean if true will test the model on the split testing set based on command line argument.
    test_model_bool = args.dont_test_model_bool

    #Number of neighbors to consider for KNN.
    knn_value = args.knn_value

    #Boolean if true will show the images as they are tested.
    show_img= args.show_img

    train_lines, test_lines = load_and_split_data(dataset_paths, data_split_ratio, image_type)
    knn_model = train_model(train_lines, model_filename, save_model_bool)
    if(test_model_bool):
        test_model(test_lines, knn_model, knn_value, show_img)

if __name__ == "__main__":
    main()