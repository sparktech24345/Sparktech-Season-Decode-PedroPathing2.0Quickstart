import cv2
import numpy as np

# -------------------------
# Global variables
# -------------------------
testVar = 0
areaTreshold = 500

# initialize all position/size variables with safe defaults
x_green = y_green = w_green = h_green = 1
x_purple = y_purple = w_purple = h_purple = 1
x_final_p = y_final_p = w_final_p = h_final_p = 1


# -------------------------
# Helper functions
# -------------------------
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("testvar:", testVar)


def drawDecorations(image):
    cv2.putText(
        image,
        '',
        (0, 230),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        1,
        cv2.LINE_AA
    )


# -------------------------
# Main pipeline
# -------------------------
def runPipeline(image, llrobot):
    global x_green, y_green, w_green, h_green
    global x_purple, y_purple, w_purple, h_purple
    global areaTreshold  # verifica un minim al ariei

    # Local safe defaults (prevents UnboundLocalError)
    x_final_p = y_final_p = w_final_p = h_final_p = 1
    x_green = y_green = w_green = h_green = 1

    # Convert to HSV
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Color masks (tune thresholds as needed)
    img_threshold_purple = cv2.inRange(img_hsv, (125, 70, 70), (160, 255, 255))
    img_threshold_green = cv2.inRange(img_hsv, (60, 200, 110), (100, 255, 255))

    # -----------------------------------
    # PURPLE detection
    # -----------------------------------
    contours_p, _ = cv2.findContours(img_threshold_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    if len(contours_p) > 0:
        cv2.drawContours(image, contours_p, -1, 255, 2)
        largestContour = max(contours_p, key=cv2.contourArea)

        y_max_purple = 0
        bottom_purple = 0

        # choose the lowest eligible contour (largest y)
        for contour in contours_p:
            x_purple, y_purple, w_purple, h_purple = cv2.boundingRect(contour)
            if (y_max_purple < y_purple) and (w_purple * h_purple >= areaTreshold):
                y_max_purple = y_purple

        # draw and finalize only if we actually found a matching contour
        for contour in contours_p:
            x_purple, y_purple, w_purple, h_purple = cv2.boundingRect(contour)
            if (y_max_purple == y_purple) and (w_purple * h_purple >= areaTreshold):
                cv2.rectangle(
                    image,
                    (x_purple, y_purple),
                    (x_purple + w_purple, y_purple + h_purple),
                    (255, 0, 255),
                    2
                )
                x_final_p = x_purple
                y_final_p = y_purple
                w_final_p = w_purple
                h_final_p = h_purple
                bottom_purple = y_purple + h_purple / 2
                print("Purple:", bottom_purple)

    incrementTestVar()
    drawDecorations(image)

    # -----------------------------------
    # GREEN detection
    # -----------------------------------
    contours_g, _ = cv2.findContours(img_threshold_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_g) > 0:
        cv2.drawContours(image, contours_g, -1, 255, 2)
        largestContour = max(contours_g, key=cv2.contourArea)
        x_green, y_green, w_green, h_green = cv2.boundingRect(largestContour)

        y_min_green = 0
        for contour in contours_g:
            xg, yg, wg, hg = cv2.boundingRect(contour)
            if (y_min_green < yg) and (wg * wg >= areaTreshold):
                y_min_green = yg

        for contour in contours_g:
            xg, yg, wg, hg = cv2.boundingRect(contour)
            if (y_min_green == yg) and (areaTreshold < hg * wg):
                cv2.rectangle(
                    image,
                    (xg, yg),
                    (xg + wg, yg + hg),
                    (90, 0, 255),
                    2
                )
                bottom_green = yg + hg / 2
                x_green, y_green, w_green, h_green = xg, yg, wg, hg
                print("Green:", bottom_green)

    incrementTestVar()
    drawDecorations(image)

    # -----------------------------------
    # Final safety (no crash even if nothing detected)
    # -----------------------------------
    if x_final_p is None:
        x_final_p = 1

    llpython = [
        x_final_p, y_final_p, h_final_p, w_final_p,
        x_green, y_green, h_green, w_green
    ]

    return largestContour, image, llpython


# -------------------------
# Example usage (optional)
# -------------------------
if __name__ == "__main__":
    # Dummy test frame to ensure code runs
    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
    llrobot = None

    contour, img_out, data = runPipeline(dummy_image, llrobot)
    print("Output data:", data)
