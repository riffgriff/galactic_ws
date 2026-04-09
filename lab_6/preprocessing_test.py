import cv2

imgs_base_path = r"C:\Users\griff\OneDrive\Desktop\Spring 26\Intro to Robotics\galactic_ws\lab_6\2026S_imgs\\"
total_images = 153
image_type = ".png"

original_images = list(cv2.imread(f"{imgs_base_path}{i:03d}{image_type}") for i in range(total_images))

cv2.imshow("image_go_brr", original_images[44])
cv2.waitKey(0)
cv2.destroyAllWindows()