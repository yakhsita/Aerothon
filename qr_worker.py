# this is for the qr containing the coordinate for next qr, NOT the target QR

def qr_worker():
    while True:
        try:
            data, width, height = raw_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        # Reshape here, off the callback thread
        try:
            img   = np.frombuffer(data, dtype=np.uint8)
            frame = img.reshape((height, width, 3)).copy()
            frame, corridor_found, offset = detect_corridor(frame)
            if corridor_found:
                print(f"Corridor Offset = {offset}")
        except Exception as e:
            print(f"Reshape error: {e}")
            continue

        # QR decode only when mission active
        if current_state == MissionState.SCAN_START_QR:
            try:
                qr_codes = decode(frame)
                for qr in qr_codes:
                    x, y, w, h = qr.rect
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    text = qr.data.decode("utf-8")
                    cv2.putText(frame, text, (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if text in visited:
                        continue

                    print(f"QR Detected: {text}")
                    visited.add(text)

                    if text.strip().upper() == "LAND":
                        land()
                        break

                    try:
                        parts = text.split(",")
                        gz_x  = float(parts[0].strip())
                        gz_y  = float(parts[1].strip())

                        ned_x, ned_y, ned_z = gazebo_to_ned(gz_x, gz_y, CRUISE_ALTITUDE)
                        print(f"Gazebo ({gz_x}, {gz_y}) → NED ({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})")

                        def navigate(nx=ned_x, ny=ned_y, nz=ned_z):
                            global moving
                            moving = True
                            goto_position(nx, ny, nz)
                            wait_until_reached(nx, ny)
                            moving = False
                            print("Ready for next QR\n")

                        target_delivery = (ned_x, ned_y)
                        change_state(MissionState.FIND_GREEN_BANNER)
                        print(f"Stored destination: {current_target}")
                        print("Searching for corridor...")

                    except Exception as e:
                        print(f"  Invalid QR format: {e}")

            except Exception as e:
                print(f"QR decode error: {e}")

        # Send annotated frame to display queue
        try:
            display_queue.put_nowait(frame)
        except queue.Full:
            pass

threading.Thread(target=qr_worker, daemon=True).start()
