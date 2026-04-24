import cv2
import math
import random
from pathlib import Path

def apply_filters(
        image,
        blur_size,
        canny_threshold1,
        canny_threshold2,
        dilate_iterations,
        close_kernel_size,
    ):
    blur = cv2.GaussianBlur(image, (blur_size, blur_size), sigmaX=0, sigmaY=0)
    img = cv2.Canny(blur, threshold1=canny_threshold1, threshold2=canny_threshold2)
    
    # Strengthen edges with light dilation
    if dilate_iterations > 0:
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        img = cv2.dilate(img, dilate_kernel, iterations=dilate_iterations)
    
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (close_kernel_size, close_kernel_size))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, close_kernel)
    
    return img

def get_bounding_box(
        image,
        blur_size=7, # gaussian blur kernel size
        canny_threshold1=20, # lower Canny threshold (catches more edges)
        canny_threshold2=50, # upper Canny threshold
        dilate_iterations=1, # dilate edges to strengthen them
        close_kernel_size=3, # kernel size for closing operation
        avg_central_edge_size=1200, # expected size of the central object
        min_central_edge_size=400, # minimum size of central object
        size_vs_location=0.95, # weighting of size and location in object selection - 0 is all size, 1 is all location
        min_area=5000,
        max_area=50000,
        min_aspect_ratio=0.5,
        max_aspect_ratio=4.0
    ):
    '''
    takes a cv2 image
    returns (x, y, w, h, edge_mask)
        x, y is the top left coordinate of the bounding box
        w, h is the width and height of the bounding box
        edge_mask is a cv2 mask of the edges for debugging purposes
    '''
    # get the center pixel of the image
    image_center = (int(image.shape[1]/2), int(image.shape[0]/2))

    # tuneable parameters in this model:

    edge_mask = apply_filters(
        image,
        blur_size=blur_size,
        canny_threshold1=canny_threshold1,
        canny_threshold2=canny_threshold2,
        dilate_iterations=dilate_iterations,
        close_kernel_size=close_kernel_size,
    )

    # find which connected edge is large and in the center
    # lower score is better
    total_edges, _, stats, centroids = cv2.connectedComponentsWithStats(edge_mask, connectivity=8)
    sizes = stats[:,-1]
    best_score = 1000000000
    best_edge = -1
    for j in range(total_edges):
        if sizes[j] < min_central_edge_size:
            continue
        centroid = (int(centroids[j][0]), int(centroids[j][1]))
        size_score = abs(sizes[j] - avg_central_edge_size)
        dist_score = math.hypot((centroid[0] - image_center[0]), abs(centroid[1] - image_center[1]))
        tot_score = (1 - size_vs_location)*size_score + size_vs_location*dist_score
        if tot_score < best_score:
            best_score = tot_score
            best_edge = j

    x, y, w, h = stats[best_edge, 0:4]

    return x, y, w, h, edge_mask

def is_reasonable_box(w, h,
        min_area=5000,
        max_area=100000,
        min_aspect_ratio=0.5,
        max_aspect_ratio=4.0):
    
    aspect_ratio = w/h
    area = w*h

    if aspect_ratio < min_aspect_ratio or\
       aspect_ratio > max_aspect_ratio or\
       area < min_area or\
       area > max_area:
        return False
    else:
        return True


if __name__ == "__main__":
    # display a random selection of the images with bounding boxes

    imgs_base_path = Path(__file__).resolve().parent / "2026S_imgs"
    total_images = 153
    image_type = ".png"

    original_images = list(cv2.imread(str(imgs_base_path / f"{i:03d}{image_type}")) for i in range(total_images))

    # for i, og_img in enumerate(original_images):
    img_num = 0
    i = 0
    while img_num < total_images:
        og_img = original_images[img_num]

        x, y, w, h, edge_mask = get_bounding_box(og_img)

        cv2.rectangle(og_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # aspect_ratio = w/h
        # area = w*h
        # print(f"Aspect ratio: {aspect_ratio}\tArea: {area}")
        if is_reasonable_box(w, h):
            print("Reasonable")
        else:
            print("Unreasonable")
        cv2.imshow("Original", og_img)
        cv2.imshow("Edge mask", edge_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        img_num += random.randint(1,20)
        i += 1