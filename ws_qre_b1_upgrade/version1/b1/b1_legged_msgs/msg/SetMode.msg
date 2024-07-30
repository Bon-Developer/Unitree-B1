uint8 mode  # 0. idle, default stand  1. force stand (controlled by dBodyHeight + ypr)
            # 2. target velocity walking (controlled by velocity + yawSpeed)
            # 3. target position walking (controlled by position + ypr[0])
            # 4. path mode walking (reserve for future release)
            # 5. position stand down. 
            # 6. position stand up 
            # 7. damping mode 
            # 8. recovery stand
            # 9. backflip
            # 10. jumpYaw
            # 11. straightHand
            # 12. dance1
            # 13. dance2
uint8 speed_level # 0. default low speed.
                  # 1. medium speed 
                  # 2. high speed. during walking, only respond MODE 3
uint8 gait_type   # 0.idle
                  # 1.trot
                  # 2.trot running
                  # 3.climb stair
                  # 4.trot obstacle  