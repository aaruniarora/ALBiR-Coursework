import sensor
from servos import Servo
from camera import Cam
import time

servo = Servo()
servo.soft_reset()

thresholds = [
    (40, 56, -29, -9, 0, 26), # Dark green
    (50, 61, -17, 4, -35, -15), # Blue
    (85, 92, -13, 5, 10, 45), # Yellow
    (49, 61, 19, 35, 9, 32), # Red
    (35, 48, 5, 20, -15, 7), # Purple
    (54, 67, 7, 22, 5, 37), # Orange
    (67, 77, -26, -6, 8, 34), # Light green
]

camera = Cam(thresholds, gain=30)

STOP_DISTANCE = 350  # Minimum y-value for distance
current_blob_idx = 0      # Starting blob index

def move_servos(left_speed, right_speed, duration=200):
    servo.set_speed(left_speed, right_speed)
    time.sleep_ms(duration)
    servo.set_speed(0, 0)

while current_blob_idx < len(thresholds):
    img = sensor.snapshot()

    # Find white blobs and color blobs
    white_blob = img.find_blobs([(84, 96, -3, 8, -11, 3)], pixels_threshold=20000, area_threshold=20000)
    blobs = img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150)

    # Identify the relevant blob of current color
    colours = camera.get_blob_colours(blobs)
    print('Colours:',colours, 'current blob idx', current_blob_idx, 'looking for num:', pow(2, current_blob_idx))

    # Draw rectangles for debugging
    for b in white_blob:
        img.draw_rectangle(b.rect())
    for b in blobs:
        img.draw_rectangle(b.rect())

    if pow(2, current_blob_idx) not in colours:
        # Search for the blob
        print('searching')
        move_servos(0.1, 0.1, duration=150)

        # If no white blob, we might be off paper
        if not white_blob:
            print('Turning away from edge of paper')
            servo.set_differential_drive(0.05, 1)
            time.sleep_ms(1000)
            servo.set_speed(0, 0)
        continue

    # If we found a valid blob, move towards it
    print(f'Found Blob! current blob: {current_blob_idx}, threshold: {thresholds[current_blob_idx]}')
    target_blob = camera.get_biggest_blob(blobs)
    img.draw_rectangle(target_blob.rect())  # Debug

    bx, by = target_blob.cx(), target_blob.cy()
    offset = bx - camera.w_centre

    if abs(offset) > 75:
        # Turn to center the blob horizontally
        turn_speed = 0.06 if offset < 0 else -0.06
        move_servos(turn_speed, -turn_speed, duration=80)
    else:
        # Move forward to approach blob
        move_servos(0.1, 0.1)

    # Check if we've reached the stop distance
    if by > STOP_DISTANCE:
        move_servos(0.1, 0.1, duration=350)
        print("\n###### BLOB FOUND ########\n")
        time.sleep_ms(2000)
        current_blob_idx += 1
