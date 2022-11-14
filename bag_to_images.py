#!/usr/bin/env python3

import sqlite3
import cv2
import numpy as np
import argparse
import os
from multiprocessing import Process
import glob
import os

parser = argparse.ArgumentParser()
parser.add_argument("--source", type=str, help="Source for rosbag")
parser.add_argument("--destination", type=str, help="Destination for video output")
args = parser.parse_args()


# default to true for latest driver version
CONVERT_BGR = True


def vimba_as_numpy_ndarray(
    byteArray, height=772, width=1032, bits_per_channel=8, bitsPerPixel=10
):
    channels_per_pixel = bitsPerPixel // bits_per_channel
    return np.ndarray(
        shape=(height, width, channels_per_pixel),
        buffer=byteArray,
        dtype=np.uint8 if bits_per_channel == 8 else np.uint16,
    )


def get_opencv_img_from_buffer(buffer, flags):
    bytes_as_np_array = np.ndarray(
        shape=(1, len(buffer)), dtype=np.uint8, buffer=buffer
    )
    img = cv2.imdecode(bytes_as_np_array, flags)
    if CONVERT_BGR:
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    else:
        return img


def readBlobData(files, destination, frame, skipCount=0, showCount=1000000):
    try:
        topicName = "/" + frame + "imageU8"

        os.makedirs(os.path.join(destination, frame), exist_ok=True)

        sqliteConnection = None
        img = None
        num_frame = 0
        topicId = -1
        verbose = False

        print("Writing " + frame)
        for dbName in files:
            sqliteConnection = sqlite3.connect(dbName)
            cursor = sqliteConnection.cursor()
            if verbose:
                print("Connected to SQLite")

            if topicName is not None:
                cursor.execute("""SELECT id from topics where name = ?""", (topicName,))
                row = cursor.fetchone()
                if row is None:
                    print(f"Topic {topicName} not found in {dbName}")
                    continue
                topicId = int(row[0])
                if verbose:
                    print("topicName: ", topicName, ", topicId: ", topicId)

            jpegMode = "compressed" in topicName

            cursor.execute(
                """SELECT * from messages where topic_id = ? LIMIT ?,?""",
                (topicId, skipCount, showCount),
            )

            done = False
            while not done:
                row = cursor.fetchone()
                if row is None:
                    done = True
                    break
                num_frame = num_frame + 1
                topic_id_from_message = row[1]

                if topic_id_from_message == topicId:
                    image_name = frame + f"_{num_frame}.jpeg"
                    final_name = os.path.join(destination, frame, image_name)

                    if not os.path.exists(final_name):
                        timestamp = row[2]
                        photo = row[3]

                        if jpegMode:
                            index = photo.find(b"JFIF")
                            jpgArray = photo[index - 6 :]  # offset where image data starts
                        else:
                            index = photo.find(b"bayer")
                            if index == -1 or index > 0x40:
                                index = photo.find(b"bgr") + 3
                            if index == -1 or index > 0x40:
                                index = 0
                            jpgArray = photo[index:]  # offset where image data starts

                        if jpegMode:
                            img = get_opencv_img_from_buffer(jpgArray, cv2.IMREAD_ANYCOLOR)
                        else:
                            # raw image format
                            frame = vimba_as_numpy_ndarray(
                                jpgArray,
                                height=1544,
                                width=2064,
                                bits_per_channel=8,
                                bitsPerPixel=24,
                            )
                            # Use opencv to convert raw Bayer image to RGB image
                            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                        cv2.imwrite(final_name, img)

        print("Finished writing " + frame)
        cursor.close()
    except sqlite3.Error as error:
        print("Failed to read blob data from sqlite table", error)
    finally:
        if sqliteConnection:
            sqliteConnection.close()
            if verbose:
                print("sqlite connection is closed")


if __name__ == "__main__":
    frames = [
        "camera_id0",
        "camera_id1",
        "camera_id2",
        "camera_id3",
        "camera_id4",
        "camera_id5",
    ]

    file_path = args.source + ".db3"
    # files = sorted(
    #     glob.glob(file_path), key=lambda x: int(x.split("_")[-1].split(".")[0])
    # )
    # print("Loading in the following bag files for processing: \n{}".format(files))

    processes = []
    for frame in frames:
        p = Process(target=readBlobData, args=([file_path], args.destination, frame))
        p.start()
        processes.append(p)

    for p in processes:
        p.join()