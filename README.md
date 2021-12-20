FP5. Performance Evaluation 1: TTC Lidar

When is the Lidar based TTC estimate way off ?

we have 2 examples where the variation of the estimated TTC is quite strong:

- Image index 3 to image index 4, the difference on xmin is almost 2 times higher compared to previous estimation. We have a difference of 11 cm compared to 6 cm in average on previous estimations. This quite high difference implies a high brake speed estimated for the preceding vehicle and a lower Time To collision of 7,12 seconds. This estimation is 45% lower than the mean of previous estimations.


![Image index 3 - 4 analysis](performance_data/imgIndex3-4_analysis.png)

we can see that a point is a little in advance from others. This might be the cause of this sudden higher difference. In the outliers removal phase, all points further than 18cm from the median value are removed. A lower threshold could fix the problem is this case.


_ Image index 6 to image index 7, the difference on xmin is very low, almost 2 times smaller compared to previous estimation.  We have a difference of 3cm compared to 6,5cm in average on previous estimations. In this case, a low difference estimated implies a low brake speed estimated for the preceding vehicle and a higher TTC of 34,34 seconds. This estimation is 274% higher than the mean of previous estimations.

![Image index 6 - 7 analysis](performance_data/imgIndex6-7_analysis.png )

We can see that the boundingboxes are very close to each other, this means that the variation in distance to the ego vehicle is very low. This explain why a such high TTC is estimated.

FP6. Performance Evaluation 2: TTC Camera

We ran all detector / descriptor combinations in order to look at differences in TTC estimation. 

We want combinations that offer a steady TTC estimation (without sudden spike or erroneous values) and with a progressive negative slope (that reflects the approach to the preceding vehicle).

We found 7 combinations that work pretty well, compared in the following graph:

![Top 7](performance_data/TTCcamera_7best.png)

 among those the best three are the following:

- AKAZE detector - SIFT descriptor
- AKAZE detector - FREAK descriptor
- AKAZE detector - AKAZE descriptor

![Top 3](performance_data/TTCcamera_3best.png)


We can observe that it is the AKAZE detector that estimates the better TTC. A ground truth is needed to be more accurate on the combinations comparaison and finding the best combination.
From Middle Term Project, we have seen that the AKAZE detector is quite slow. In the case we want to choose a combination applicable in a embedded environment, it is more suitable to pick the following combination:

- SIFT detector - BRIEF descriptor

This combination is in top 7 without using AKAZE detector.