import cv2
import numpy as np
import serial
directionPREV = None
directionFRAME  = 0
directionFRAME_threshold = 2 

def regionOI(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def houghLines(img, rho, theta, threshold, min_line_len, max_line_gap):
    return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

def drawLines(img, lines, direction_label):
    if lines is None:
        return
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, direction_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def directionDET(lines, img_width):
    if lines is None or len(lines) == 0:
        return "NO lines" 
    
    leftLines = []
    rightLines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x1 == x2:
            continue 
        slope = (y2 - y1) / (x2 - x1)
      
        if slope < -0.5:
            rightLines.append(line)
        elif slope > 0.5:
            leftLines.append(line)
    
    if leftLines and rightLines:
        return "Forward"
    elif leftLines:
        return "Left Turn"
    elif rightLines:
        return "Right Turn"
    
    return "NO lines"

def BEV(frame):
    width, height = 1920, 1080
    pst1 = np.float32([
        [0, 1080],
        [1920, 1080],
        [1632, 648],
        [288, 648]
    ])
    pst2 = np.float32([
        [0, height],         
        [width, height], 
        [width, 0],            
        [0, 0]
    ])
    M = cv2.getPerspectiveTransform(pst1, pst2)
    warped = cv2.warpPerspective(frame, M, (width, height))
    return warped

def processed(frame):
    global directionPREV, directionFRAME, directionFRAME_threshold

    height, width = frame.shape[:2]
    
    birdseye = BEV(frame)

    grey = cv2.cvtColor(birdseye, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(grey, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    heightS = int(height * 0.4)
    widthS = int(width * 0.8)
    vertices = np.array([[
        (int((width - widthS) / 2), height),
        (int((width + widthS) / 2), height),
        (int((width + widthS) / 2), height - heightS),
        (int((width - widthS) / 2), height - heightS)
    ]], dtype=np.int32)

    cv2.polylines(birdseye, [vertices], isClosed=True, color=(0, 255, 255), thickness=2)
    roi = regionOI(edges, vertices)

    lines = houghLines(roi, 1, np.pi / 180, 15, 10, 20)
    direction = directionDET(lines, width)

    if direction == directionPREV:
        directionFRAME += 1
        if directionFRAME >= directionFRAME_threshold:
            print(f"Determined direction: {direction}")
            ser.write(direction.encode())
            directionFRAME = 0

    else:
        directionPREV = direction
        directionFRAME = 0

    drawLines(birdseye, lines, direction)
    
    return birdseye

if __name__ == "__main__":

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    cap = cv2.VideoCapture('/Users/charliefraser/Downloads/IMG_0897 2.MOV')

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('nothing.mp4', fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        processedFrame = processed(frame)
        out.write(processedFrame)
        cv2.imshow('Lane Detection', processedFrame)
        
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break
    
    cap.release()
    out.release()
    cv2.destroyAllWindows()
