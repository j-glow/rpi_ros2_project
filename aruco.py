import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    cv2.imshow('frame',frame)

    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    
    markerCorners, markerIds, _ = detector.detectMarkers(frame)

    #draw found markers
    arucoImg = frame
    
    if len(markerCorners)>0:
      markerIds.flatten()

      for (corner, ID) in zip(markerCorners, markerIds):
        markerCorners = corner.reshape((4,2))
        (topLeft, topRight, bottomRight, bottomLeft) = markerCorners

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        #draw frame
        cv2.line(arucoImg, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(arucoImg, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(arucoImg, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(arucoImg, bottomLeft, topLeft, (0, 255, 0), 2)

        #draw center point
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(arucoImg, (cX, cY), 4, (0, 0, 255), -1)

        #draw ID
        cv2.putText(arucoImg, str(ID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow("markers found", arucoImg)
    
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break

cap.release()
cv2.destroyAllWindows()

