#!/usr/bin/env python3
import os
import sys
import argparse
import csv
import cv2
import numpy as np
import pickle
from pathlib import Path

import img_preprocessing


DEFAULT_DATA_DIR = Path(__file__).resolve().parent / "2026S_imgs"
DEFAULT_K = 3

# ------------------------------------------------------------------------------
#                  DO NOT MODIFY FUNCTION NAMES OR ARGUMENTS
# ------------------------------------------------------------------------------

def initialize_model(model_path=None):
    """
    Initialize and return your trained model.
    You MUST modify this function to load and/or construct your model.
    DO NOT change the function name or its input/output.
    
    Args:
        model_path: The path to your pretrained model file (if one is needed).
    Returns:
        model: Your trained model.
    """

    if not model_path:
        raise ValueError("model_path is required. Provide a trained model file (.xml, .pkl, or .pickle).")

    model_file = Path(model_path)
    if model_file.suffix.lower() in {".pkl", ".pickle"}:
        with model_file.open("rb") as f:
            return pickle.load(f)
    if model_file.suffix.lower() == ".xml":
        knn = cv2.ml.KNearest_load(str(model_file))
        return {"kind": "opencv_knn", "model": knn, "k": DEFAULT_K, "input_size": (25, 33)}

    raise ValueError("Unsupported model format. Use .xml, .pkl, or .pickle")

def predict(model, image):
    """
    Run inference on a single image using your model.
    You MUST modify this function to perform prediction.
    DO NOT change the function signature.
    
    Args:
        model: The model object returned by initialize_model().
        image: The input image (as a NumPy array) to classify.
    
    Returns:
        int: The predicted class label.
    """

    x, y, w, h, _ = img_preprocessing.get_bounding_box(image)
    if img_preprocessing.is_reasonable_box(w,h):
        image = image[y:y+h,x:x+w] # crop image

    feature = image.astype(np.float32).flatten() / 255.0

    if isinstance(model, dict):
        kind = model.get("kind")

        if kind == "opencv_knn":
            k = int(model.get("k", DEFAULT_K))
            input_size = model.get("input_size", (25, 33))
            resized = cv2.resize(image, input_size)
            sample = resized.flatten().reshape(1, input_size[0] * input_size[1] * 3).astype(np.float32)
            ret, results, neighbours, dist = model["model"].findNearest(sample, k)
            return int(ret)

    if hasattr(model, "predict"):
        predicted = model.predict(feature.reshape(1, -1))
        if isinstance(predicted, tuple):
            predicted = predicted[0]
        return int(np.asarray(predicted).ravel()[0])

    raise TypeError(f"Unsupported model type: {type(model)!r}")

# ------------------------------------------------------------------------------
#                      DO NOT MODIFY ANY CODE BELOW THIS LINE
# ------------------------------------------------------------------------------

def load_validation_data(data_path):
    """
    Load validation images and labels from the given directory.
    Expects a 'labels.txt' file in the directory and images in .png format.
    
    Args:
        data_path (str): Path to the validation dataset.
    
    Returns:
        list of tuples: Each tuple contains (image_path, true_label)
    """
    labels_file = os.path.join(data_path, "labels.txt")
    data = []
    with open(labels_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Assumes row[0] is the image filename (without extension) and row[1] is the label.
            image_file = os.path.join(data_path, row[0] + ".png")  # Modify if images use a different extension.
            data.append((image_file, int(row[1])))
    return data

def evaluate_model(model, validation_data):
    """
    Evaluate the model on the validation dataset.
    Computes and prints the confusion matrix and overall accuracy.
    
    Args:
        model: The model object.
        validation_data (list): List of tuples (image_path, true_label).
    """
    num_classes = 6  # Number of classes (adjust if needed)
    confusion_matrix = np.zeros((num_classes, num_classes), dtype=np.int32)
    correct = 0
    total = len(validation_data)
    
    for image_path, true_label in validation_data:
        # Read the image
        image = cv2.imread(image_path)
        if image is None:
            print("Warning: Could not load image:", image_path)
            continue
        # Get the predicted label using the student's implementation.
        predicted_label = predict(model, image)
        
        if predicted_label == true_label:
            correct += 1
        confusion_matrix[true_label][predicted_label] += 1
        print(f"Image: {os.path.basename(image_path)} - True: {true_label}, Predicted: {predicted_label}")
    
    accuracy = correct / total if total > 0 else 0
    print("\nTotal accuracy:", accuracy)
    print("Confusion Matrix:")
    print(confusion_matrix)

def main():
    parser = argparse.ArgumentParser(description="Model Grader for Lab 6")
    parser.add_argument("--data_path", type=str, required=True,
                        help="Path to the validation dataset directory (must contain labels.txt and images)")
    parser.add_argument("--model_path", type=str, required=False,
                        help="Path to the trained model file (if applicable)")
    args = parser.parse_args()
    
    # Path to the validation dataset directory from command line argument.
    VALIDATION_DATASET_PATH = args.data_path

    # Path to the trained model file from command line argument.
    MODEL_PATH = args.model_path
    
    # Load validation data.
    validation_data = load_validation_data(VALIDATION_DATASET_PATH)
    
    # Initialize the model using the student's implementation.
    model = initialize_model(MODEL_PATH) if MODEL_PATH else initialize_model()
    
    # Evaluate the model on the validation dataset.
    evaluate_model(model, validation_data)

if __name__ == "__main__":
    main()
