import cv2
import numpy as np
import os
import random

# // create folder with data set
digit= "5"
base_dir="dataset"
digit_dir=os.path.join(base_dir,digit)

os.makedirs(digit_dir,exist_ok=True)

fonts = [
   cv2.FONT_HERSHEY_SIMPLEX,
   cv2.FONT_HERSHEY_PLAIN,
   cv2.FONT_HERSHEY_DUPLEX,
   cv2.FONT_HERSHEY_COMPLEX,
   cv2.FONT_HERSHEY_TRIPLEX,
   cv2.FONT_HERSHEY_COMPLEX_SMALL,
   cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
   cv2.FONT_HERSHEY_SCRIPT_COMPLEX
]


img_size= 32
samples_per_digit=1000

print("Генерация дата сета for 5")

for i in range(samples_per_digit):

    img=np.zeros((img_size,img_size),dtype=np.uint8)

    font=random.choice(fonts)
    scale=random.uniform(0.8,1.3)
    thinckeness= random.randint(1,3)

    text=digit


    (text_w,text_h),baseline=cv2.getTextSize(text,font,scale,thinckeness)


    # random smeshenie


    x=(img_size-text_w)//2+ random.randint(-2,2)
    y=(img_size+text_h)//2+random.randint(-2,2)

    # print number on pictures

    cv2.putText(img,text,(x,y),font,scale,255,thinckeness,cv2.LINE_AA)

    # save fali

    filename=os.path.join(digit_dir,f"{digit}_{i}.png")
    cv2.imwrite(filename, img)


print(f"Ready!Generated {samples_per_digit}  pictures.\n Find in folder: {digit_dir}")
