import cv2

imgs_base_path = r"C:\Users\griff\OneDrive\Desktop\Spring 26\Intro to Robotics\galactic_ws\lab_6\2026S_imgs\\"
total_images = 153
image_type = ".png"

original_images = list(cv2.imread(f"{imgs_base_path}{i:03d}{image_type}") for i in range(total_images))
edge_masks = list()
final_images = list()

# get the center pixel of the image
image_center = (int(original_images[0].shape[0]/2), int(original_images[0].shape[1]/2))

# tuneable parameters in this model:
blur_size = 7 # gaussian blur kernel size
close_kernel_size = 5 # kernel size for the closing operation
avg_central_edge_size = 100 # expected size of the central object
size_vs_location = 0.5 # weighting of size and location in object selection - 0 is all size, 1 is all location

for i, og_img in enumerate(original_images):
    # add blur for better edge detection
    img = cv2.GaussianBlur(og_img, (blur_size,blur_size), sigmaX=0, sigmaY=0)

    # edge detection
    img = cv2.Canny(img, threshold1=50, threshold2=150)

    # closing operation to connect edges
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel_size, close_kernel_size))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, close_kernel)

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
im_to_show = 116

cv2.imshow("Original", original_images[im_to_show])
cv2.imshow("Edges", edge_masks[im_to_show])
cv2.imshow("Final", final_images[im_to_show])
cv2.waitKey(0)
cv2.destroyAllWindows()