import cv2
from pathlib import Path
import random

def preprocess_image(
    image,
    blur_size=5,
    canny_threshold1=20,
    canny_threshold2=70,
    dilate_iterations=1,
    close_kernel_size=3,
):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (blur_size, blur_size), sigmaX=0, sigmaY=0)
    img = cv2.Canny(gray, threshold1=canny_threshold1, threshold2=canny_threshold2)
    
    # Strengthen edges with light dilation
    if dilate_iterations > 0:
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        img = cv2.dilate(img, dilate_kernel, iterations=dilate_iterations)
    
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel_size, close_kernel_size))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, close_kernel)
    
    return img

imgs_base_path = Path(__file__).resolve().parent / "2026S_imgs"
total_images = 153
image_type = ".png"

original_images = list(cv2.imread(str(imgs_base_path / f"{i:03d}{image_type}")) for i in range(total_images))
edge_masks = list()
final_images = list()

# get the center pixel of the image
image_center = (int(original_images[0].shape[0]/2), int(original_images[0].shape[1]/2))

# tuneable parameters in this model:
blur_size = 7 # gaussian blur kernel size
canny_threshold1 = 20 # lower Canny threshold (catches more edges)
canny_threshold2 = 70 # upper Canny threshold
dilate_iterations = 1 # dilate edges to strengthen them
close_kernel_size = 3 # kernel size for closing operation
avg_central_edge_size = 100 # expected size of the central object
size_vs_location = 0.5 # weighting of size and location in object selection - 0 is all size, 1 is all location

for i, og_img in enumerate(original_images):
    img = preprocess_image(
        og_img,
        blur_size=blur_size,
        canny_threshold1=canny_threshold1,
        canny_threshold2=canny_threshold2,
        dilate_iterations=dilate_iterations,
        close_kernel_size=close_kernel_size,
    )

    edge_masks.append(img)

    # find which connected edge is large and in the center
    total_edges, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
    sizes = stats[:,-1]
    best_score = 1000000000
    best_edge = -1
    for j in range(total_edges):
        size_score = abs(sizes[j] - avg_central_edge_size)
        dist_score = abs(centroids[j][0] - image_center[0]) + abs(centroids[j][1] - image_center[1])
        tot_score = (1 - size_vs_location)*size_score + size_vs_location*dist_score
        if tot_score < best_score:
            best_score = tot_score
            best_edge = j

    x, y, w, h = stats[best_edge, 0:4]
    # add box to the image
    cv2.rectangle(og_img, (x, y), (x + w, y + h), (0, 0, 255), 2)  # red with thickness 2

    final_images.append(og_img)

# which image gets displayed in the windows
im_to_show = 0

while im_to_show < 152:
    cv2.imshow("Original", original_images[im_to_show])
    cv2.imshow("Edges", edge_masks[im_to_show])
    cv2.imshow("Final", final_images[im_to_show])
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    im_to_show += random.randint(1,20)