#include "p2os_config.hpp"
#include <robot_params.hpp>
#include <sip.hpp>

void SIP::FillStandard(nav_msgs::ros_p2os_data_t* data) {
    ///////////////////////////////////////////////////////////////
    // odometry

    double px, py, pa;

    // initialize position to current offset
    px = this->x_offset / 1e3;
    py = this->y_offset / 1e3;
    // now transform current position by rotation if there is one
    // and add to offset
    if (this->angle_offset != 0) {
        double rot = DTOR(this->angle_offset);  // convert rotation to radians
        px += ((this->xpos / 1e3) * cos(rot) - (this->ypos / 1e3) * sin(rot));
        py += ((this->xpos / 1e3) * sin(rot) + (this->ypos / 1e3) * cos(rot));
        pa = DTOR(this->angle_offset + angle);
    } else {
        px += this->xpos / 1e3;
        py += this->ypos / 1e3;
        pa = DTOR(this->angle);
    }

    // timestamps get set in the p2os::StandardSIPPutData fcn
    // data->position.header.frame_id = odom_frame_id;
    data->position.child_frame_id = base_link_frame_id;

    data->position.pose.pose.position.x = px;
    data->position.pose.pose.position.y = py;
    data->position.pose.pose.position.z = 0.0;
    // data->position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pa);

    // add rates
    data->position.twist.twist.linear.x = ((lvel + rvel) / 2) / 1e3;
    data->position.twist.twist.linear.y = 0.0;
    data->position.twist.twist.angular.z =
        static_cast<double>((rvel - lvel) / (2.0 / PlayerRobotParams[param_idx].DiffConvFactor));

    data->position.pose.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                      0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

    data->position.twist.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                       0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

    // publish transform
    // data->odom_trans.header = data->position.header;
    data->odom_trans.child_frame_id = data->position.child_frame_id;
    data->odom_trans.transform.translation.x = px;
    data->odom_trans.transform.translation.y = py;
    data->odom_trans.transform.translation.z = 0;
    // data->odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pa);

    // battery
    data->batt.voltage = battery / 10.0;

    // motor state
    // The below will tell us if the motors are currently moving or not, it does
    // not tell us whether they have been enabled
    // data->motors.state = (status & 0x01);
    data->motors.state = motors_enabled & 0x01;
    /*
    ///////////////////////////////////////////////////////////////
    // compass
    memset(&(data->compass),0,sizeof(data->compass));
    data->compass.pos.pa = DTOR(this->compass);
    */

    ///////////////////////////////////////////////////////////////
    // sonar
    data->sonar.ranges_count = static_cast<int>(sonarreadings);
    data->sonar.ranges.clear();
    for (int i = 0; i < data->sonar.ranges_count; i++) {
        data->sonar.ranges.emplace_back(sonars[i] / 1e3);
    }

    ///////////////////////////////////////////////////////////////
    // gripper
    unsigned char gripState = timer;
    if ((gripState & 0x01) && (gripState & 0x02) && !(gripState & 0x04)) {
        data->gripper.grip.state = PLAYER_GRIPPER_STATE_ERROR;
        data->gripper.grip.dir = 0;
    } else if (gripState & 0x04) {
        data->gripper.grip.state = PLAYER_GRIPPER_STATE_MOVING;
        if (gripState & 0x01) {
            data->gripper.grip.dir = 1;
        }
        if (gripState & 0x02) {
            data->gripper.grip.dir = -1;
        }
    } else if (gripState & 0x01) {
        data->gripper.grip.state = PLAYER_GRIPPER_STATE_OPEN;
        data->gripper.grip.dir = 0;
    } else if (gripState & 0x02) {
        data->gripper.grip.state = PLAYER_GRIPPER_STATE_CLOSED;
        data->gripper.grip.dir = 0;
    }

    // Reset data to false
    data->gripper.grip.inner_beam = false;
    data->gripper.grip.outer_beam = false;
    data->gripper.grip.left_contact = false;
    data->gripper.grip.right_contact = false;

    if (digin & 0x08) {
        data->gripper.grip.inner_beam = true;
    }
    if (digin & 0x04) {
        data->gripper.grip.outer_beam = true;
    }
    if (!(digin & 0x10)) {
        data->gripper.grip.left_contact = true;
    }
    if (!(digin & 0x20)) {
        data->gripper.grip.right_contact = true;
    }

    // lift
    data->gripper.lift.dir = 0;

    if ((gripState & 0x10) && (gripState & 0x20) && !(gripState & 0x40)) {
        // In this case, the lift is somewhere in between, so
        // must be at an intermediate carry position. Use last commanded position
        data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
        data->gripper.lift.position = lastLiftPos;
    } else if (gripState & 0x40) {  // Moving
        data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_MOVING;
        // There is no way to know where it is for sure, so use last commanded
        // position.
        data->gripper.lift.position = lastLiftPos;
        if (gripState & 0x10) {
            data->gripper.lift.dir = 1;
        } else if (gripState & 0x20) {
            data->gripper.lift.dir = -1;
        }
    } else if (gripState & 0x10) {  // Up
        data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
        data->gripper.lift.position = 1.0f;
        data->gripper.lift.dir = 0;
    } else if (gripState & 0x20) {  // Down
        data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_IDLE;
        data->gripper.lift.position = 0.0f;
        data->gripper.lift.dir = 0;
    } else {  // Assume stalled
        data->gripper.lift.state = PLAYER_ACTARRAY_ACTSTATE_STALLED;
    }
    // Store the last lift position
    lastLiftPos = data->gripper.lift.position;

    /*
    ///////////////////////////////////////////////////////////////
    // bumper
    unsigned int bump_count = PlayerRobotParams[param_idx].NumFrontBumpers +
    PlayerRobotParams[param_idx].NumRearBumpers; if (data->bumper.bumpers_count != bump_count)
    {
        data->bumper.bumpers_count = bump_count;
        delete [] data->bumper.bumpers;
        data->bumper.bumpers = new uint8_t[bump_count];
    }
    int j = 0;
    for(int i=PlayerRobotParams[param_idx].NumFrontBumpers-1;i>=0;i--)
        data->bumper.bumpers[j++] =
        (unsigned char)((this->frontbumpers >> i) & 0x01);
    for(int i=PlayerRobotParams[param_idx].NumRearBumpers-1;i>=0;i--)
        data->bumper.bumpers[j++] =
        (unsigned char)((this->rearbumpers >> i) & 0x01);
    */

    ///////////////////////////////////////////////////////////////
    // digital I/O
    data->dio.count = 8;
    data->dio.bits = static_cast<unsigned int>(this->digin);

    ///////////////////////////////////////////////////////////////
    // analog I/O
    // TODO(allenh1): should do this smarter, based on which analog input is selected
    data->aio.voltages_count = static_cast<unsigned char>(1);
    // if (!data->aio.voltages)
    //   data->aio.voltages = new float[1];
    // data->aio.voltages[0] = (this->analog / 255.0) * 5.0;
    data->aio.voltages.clear();
    data->aio.voltages.push_back((this->analog / 255.0) * 5.0);
}

int SIP::PositionChange(uint16_t from, uint16_t to) {
    int diff1, diff2;

    /* find difference in two directions and pick int16_test */
    if (to > from) {
        diff1 = to - from;
        diff2 = -(from + 4096 - to);
    } else {
        diff1 = to - from;
        diff2 = 4096 - from + to;
    }
    return abs(diff1) < abs(diff2) ? diff1 : diff2;
}

void SIP::Print() {
#ifdef P2OS_DEBUG_PRINT
    int i;

    this->debug_serial->printf("debug: lwstall:%d rwstall:%d\n", lwstall, rwstall);

    String front_bumper_info = "";
    for (int i = 0; i < 5; i++) {
        front_bumper_info += " ";
        front_bumper_info += String((frontbumpers >> i) & 0x01);
        // front_bumper_info << " " <<
        // static_cast<int>((frontbumpers >> i) & 0x01 );
    }
    this->debug_serial->printf("debug: Front bumpers:%s\n", front_bumper_info.c_str());

    String rear_bumper_info = "";
    for (int i = 0; i < 5; i++) {
        rear_bumper_info += " ";
        rear_bumper_info += String((rearbumpers >> i) & 0x01);
    }
    this->debug_serial->printf("debug: Rear bumpers:%s\n", rear_bumper_info.c_str());

    this->debug_serial->printf("debug: status: 0x%x analog: %d param_id: %d ", status, analog, param_idx);
    String status_info = "";
    for (i = 0; i < 11; i++) {
        status_info += " ";
        status_info += String((status >> (7 - i)) & 0x01);
    }
    this->debug_serial->printf("debug: status:%s\n", status_info.c_str());
    String digin_info = "";
    for (i = 0; i < 8; i++) {
        digin_info += " ";
        digin_info += String((digin >> (7 - i)) & 0x01);
    }
    this->debug_serial->printf("debug: digin:%s\n", digin_info.c_str());
    String digout_info = "";
    for (i = 0; i < 8; i++) {
        digout_info += " ";
        digout_info += String((digout >> (7 - i)) & 0x01);
    }
    this->debug_serial->printf("debug: digout:%s\n", digout_info.c_str());
    this->debug_serial->printf("debug: battery: %d compass: %d sonarreadings: %d\n", battery, compass, sonarreadings);
    this->debug_serial->printf("debug: xpos: %d ypos:%d ptu:%hu timer:%hu\n", xpos, ypos, ptu, timer);
    this->debug_serial->printf("debug: angle: %d lvel: %d rvel: %d control: %d\n", angle, lvel, rvel, control);

    PrintSonars();
    PrintArmInfo();
    PrintArm();
#endif
}

void SIP::PrintSonars() {
    if (this->sonarreadings <= 0) {
        return;
    }
    String sonar_info;
    for (int i = 0; i < this->sonarreadings; i++) {
        sonar_info += " ";
        sonar_info += static_cast<int>(sonars[i]);
    }
    this->debug_serial->printf("debug: Sonars: %s\n", sonar_info.c_str());
}

void SIP::PrintArm() {
#ifdef P2OS_DEBUG_PRINT
    String arm_power_state = (armPowerOn ? "on" : "off");
    String arm_connected = (armConnected ? "" : "not ");
    this->debug_serial->printf("debug: Arm power is %s\tArm is %sconnected\n", arm_power_state, arm_connected);
    this->debug_serial->printf("debug: Arm joint status:\n");
    for (int ii = 0; ii < 6; ii++) {
        String arm_moving = (armJointMoving[ii] ? "Moving " : "Stopped");
        this->debug_serial->printf("debug: Joint %d   %s   %d\n", ii + 1, arm_moving, armJointPos[ii]);
    }
    this->debug_serial->println();
#endif
}

void SIP::PrintArmInfo() {
#ifdef P2OS_DEBUG_PRINT
    this->debug_serial->printf("debug: Arm version:\t%s\n", armVersionString);
    this->debug_serial->printf("debug: Arm has %d joints:\n", armNumJoints);
    this->debug_serial->printf("debug:  |\tSpeed\tHome\tMin\tCentre\tMax\tTicks/90\n");
    for (int ii = 0; ii < armNumJoints; ii++) {
        this->debug_serial->printf(
            "%d |\t%d\t%d\t%d\t%d\t%d\t%d\n", ii, armJoints[ii].speed, armJoints[ii].home, armJoints[ii].min,
            armJoints[ii].centre, armJoints[ii].max, armJoints[ii].ticksPer90
        );
    }
#endif
}

void SIP::ParseStandard(unsigned char* buffer) {
    int      cnt = 0, change;
    uint16_t newxpos, newypos;

    status = buffer[cnt];
    cnt += sizeof(unsigned char);
    /*
     * Remember P2OS uses little endian:
     * for a 2 byte int16_t (called integer on P2OS)
     * byte0 is low byte, byte1 is high byte
     * The following code is host-machine endian independant
     * Also we must or (|) bytes together instead of casting to a
     * int16_t * since on ARM architectures int16_t * must be even byte aligned!
     * You can get away with this on a i386 since int16_ts * can be
     * odd byte aligned. But on ARM, the last bit of the pointer will be ignored!
     * The or'ing will work on either arch.
     */
    newxpos = ((buffer[cnt] | (buffer[cnt + 1] << 8)) & 0xEFFF) % 4096; /* 15 ls-bits */

    if (xpos != INT_MAX) {
        change = static_cast<int>(rint(PositionChange(rawxpos, newxpos) * PlayerRobotParams[param_idx].DistConvFactor));
        if (abs(change) > 100) {
#ifdef P2OS_ERROR_PRINT
            this->debug_serial->printf("error: invalid odometry change [%d]; odometry values are tainted\n", change);
#endif
        } else {
            xpos += change;
        }
    } else {
        xpos = 0;
    }
    rawxpos = newxpos;
    cnt += sizeof(int16_t);

    newypos = ((buffer[cnt] | (buffer[cnt + 1] << 8)) & 0xEFFF) % 4096; /* 15 ls-bits */

    if (ypos != INT_MAX) {
        change = static_cast<int>(rint(PositionChange(rawypos, newypos) * PlayerRobotParams[param_idx].DistConvFactor));
        if (abs(change) > 100) {
#ifdef P2OS_ERROR_PRINT
            this->debug_serial->printf("error: invalid odometry change [%d]; odometry values are tainted\n", change);
#endif
        } else {
            ypos += change;
        }
    } else {
        ypos = 0;
    }
    rawypos = newypos;
    cnt += sizeof(int16_t);

    angle = (int16_t)rint(
        ((int16_t)(buffer[cnt] | (buffer[cnt + 1] << 8))) * PlayerRobotParams[param_idx].AngleConvFactor * 180.0 / M_PI
    );
    cnt += sizeof(int16_t);

    lvel =
        (int16_t)rint(((int16_t)(buffer[cnt] | (buffer[cnt + 1] << 8))) * PlayerRobotParams[param_idx].VelConvFactor);
    cnt += sizeof(int16_t);

    rvel =
        (int16_t)rint(((int16_t)(buffer[cnt] | (buffer[cnt + 1] << 8))) * PlayerRobotParams[param_idx].VelConvFactor);
    cnt += sizeof(int16_t);

    battery = buffer[cnt];
    cnt += sizeof(unsigned char);
#ifdef P2OS_DEBUG_PRINT
    this->debug_serial->printf("debug: battery value: %d\n", battery);
#endif

    lwstall = buffer[cnt] & 0x01;
    rearbumpers = buffer[cnt] >> 1;
    cnt += sizeof(unsigned char);

    rwstall = buffer[cnt] & 0x01;
    frontbumpers = buffer[cnt] >> 1;
    cnt += sizeof(unsigned char);

    control =
        (int16_t)rint(((int16_t)(buffer[cnt] | (buffer[cnt + 1] << 8))) * PlayerRobotParams[param_idx].AngleConvFactor);
    cnt += sizeof(int16_t);

    ptu = (buffer[cnt] | (buffer[cnt + 1] << 8));
    motors_enabled = buffer[cnt];
    sonar_flag = buffer[cnt + 1];
    cnt += sizeof(int16_t);

    // compass = buffer[cnt]*2;
    if (buffer[cnt] != 255 && buffer[cnt] != 0 && buffer[cnt] != 181) {
        compass = (buffer[cnt] - 1) * 2;
    }
    cnt += sizeof(unsigned char);

    unsigned char numSonars = buffer[cnt];
    cnt += sizeof(unsigned char);

    if (numSonars > 0) {
        // find the largest sonar index supplied
        unsigned char maxSonars = sonarreadings;
        for (unsigned char i = 0; i < numSonars; i++) {
            unsigned char sonarIndex = buffer[cnt + i * (sizeof(unsigned char) + sizeof(uint16_t))];
            if ((sonarIndex + 1) > maxSonars) {
                maxSonars = sonarIndex + 1;
            }
        }

        // if necessary make more space in the array and preserve existing readings
        if (maxSonars > sonarreadings) {
            uint16_t* newSonars = new uint16_t[maxSonars];
            for (unsigned char i = 0; i < sonarreadings; i++) {
                newSonars[i] = sonars[i];
            }
            if (sonars != NULL) {
                delete[] sonars;
            }
            sonars = newSonars;
            sonarreadings = maxSonars;
        }

        // update the sonar readings array with the new readings
        for (unsigned char i = 0; i < numSonars; i++) {
            sonars[buffer[cnt]] = static_cast<uint16_t>(
                rint((buffer[cnt + 1] | (buffer[cnt + 2] << 8)) * PlayerRobotParams[param_idx].RangeConvFactor)
            );
            cnt += sizeof(unsigned char) + sizeof(uint16_t);
        }
    }

    timer = (buffer[cnt] | (buffer[cnt + 1] << 8));
    cnt += sizeof(int16_t);

    analog = buffer[cnt];
    cnt += sizeof(unsigned char);

    digin = buffer[cnt];
    cnt += sizeof(unsigned char);

    digout = buffer[cnt];
    cnt += sizeof(unsigned char);
    // for debugging:
    Print();
    // PrintSonars();
}

/** Parse a SERAUX SIP packet.  For a CMUcam, this will have blob
 **  tracking messages in the format (all one-byte values, no spaces):
 **
 **      255 M mx my x1 y1 x2 y2 pixels confidence  (10-bytes)
 **
 ** Or color info messages of the format:
 **
 **      255 S Rval Gval Bval Rvar Gvar Bvar    (8-bytes)
 */
void SIP::ParseSERAUX(unsigned char* buffer) {
    unsigned char type = buffer[1];
    if (type != SERAUX && type != SERAUX2) {
// Really should never get here...
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Attempt to parse non SERAUX packet as serial data.\n");
#endif
        return;
    }

    int len = static_cast<int>(buffer[0]) - 3;  // Get the string length

    /* First thing is to find the beginning of last full packet (if possible).
    ** If there are less than CMUCAM_MESSAGE_LEN*2-1 bytes (19), we're not
    ** guaranteed to have a full packet.  If less than CMUCAM_MESSAGE_LEN
    ** bytes, it is impossible to have a full packet.
    ** To find the beginning of the last full packet, search between bytes
    ** len-17 and len-8 (inclusive) for the start flag (255).
    */
    int ix;
    for (ix = (len > 18 ? len - 17 : 2); ix <= len - 8; ix++) {
        if (buffer[ix] == 255) {
            break;  // Got it!
        }
    }
    if (len < 10 || ix > len - 8) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Failed to get a full blob tracking packet.\n");
#endif
        return;
    }

    // There is a special 'S' message containing the tracking color info
    if (buffer[ix + 1] == 'S') {
// Color information (track color)
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf(
            "Tracking color (RGB):  %d %d %d\n"
            "       with variance:  %d %d %d\n",
            buffer[ix + 2], buffer[ix + 3], buffer[ix + 4], buffer[ix + 5], buffer[ix + 6], buffer[ix + 7]
        );
        blobcolor = buffer[ix + 2] << 16 | buffer[ix + 3] << 8 | buffer[ix + 4];
#endif
        return;
    }

    // Tracking packets with centroid info are designated with a 'M'
    if (buffer[ix + 1] == 'M') {
        // Now it's easy.  Just parse the packet.
        blobmx = buffer[ix + 2];
        blobmy = buffer[ix + 3];
        blobx1 = buffer[ix + 4];
        bloby1 = buffer[ix + 5];
        blobx2 = buffer[ix + 6];
        bloby2 = buffer[ix + 7];
        blobconf = buffer[ix + 9];
        // Xiaoquan Fu's calculation for blob area (max 11297).
        blobarea = (bloby2 - bloby1 + 1) * (blobx2 - blobx1 + 1) * blobconf / 255;
        return;
    }

#ifdef P2OS_DEBUG_PRINT
    this->debug_serial->printf("debug: Unknown blob tracker packet type: %c\n", buffer[ix + 1]);
#endif
}

// Parse a set of gyro measurements.  The buffer is formatted thusly:
//     length (2 bytes), type (1 byte), count (1 byte)
// followed by <count> pairs of the form:
//     rate (2 bytes), temp (1 byte)
// <rate> falls in [0,1023]; less than 512 is CCW rotation and greater
// than 512 is CW
void SIP::ParseGyro(unsigned char* buffer) {
    // Get the message length (account for the type byte and the 2-byte
    // checksum)
    int len = static_cast<int>(buffer[0] - 3);

    unsigned char type = buffer[1];
    if (type != GYROPAC) {
// Really should never get here...
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Attempt to parse non GYRO packet as gyro data.\n");
#endif
        return;
    }

    if (len < 1) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Couldn't get gyro measurement count\n");
#endif
        return;
    }

    // get count
    int count = static_cast<int>(buffer[2]);

    // sanity check
    if ((len - 1) != (count * 3)) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Mismatch between gyro measurement count and packet length\n");
#endif
        return;
    }

    // Actually handle the rate values.  Any number of things could (and
    // probably should) be done here, like filtering, calibration, conversion
    // from the gyro's arbitrary units to something meaningful, etc.
    //
    // As a first cut, we'll just average all the rate measurements in this
    // set, and ignore the temperatures.
    float    ratesum = 0;
    int      bufferpos = 3;
    uint16_t rate;
    for (int i = 0; i < count; i++) {
        rate = (uint16_t)(buffer[bufferpos++]);
        rate |= buffer[bufferpos++] << 8;

        ratesum += rate;
    }

    int32_t average_rate = static_cast<int32_t>(rint(ratesum / static_cast<float>(count)));

    // store the result for sending
    gyro_rate = average_rate;
}

void SIP::ParseArm(unsigned char* buffer) {
    int length = static_cast<int>(buffer[0]) - 2;

    if (buffer[1] != ARMPAC) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: Attempt to parse a non ARM packet as arm data.\n");
#endif
        return;
    }

    if (length < 1 || length != 9) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: ARMpac length incorrect size\n");
#endif
        return;
    }

    unsigned char status = buffer[2];
    armPowerOn = status & 0x01;
    armConnected = status & 0x02;
    unsigned char motionStatus = buffer[3];
    if (motionStatus & 0x01) {
        armJointMoving[0] = true;
    }
    if (motionStatus & 0x02) {
        armJointMoving[1] = true;
    }
    if (motionStatus & 0x04) {
        armJointMoving[2] = true;
    }
    if (motionStatus & 0x08) {
        armJointMoving[3] = true;
    }
    if (motionStatus & 0x10) {
        armJointMoving[4] = true;
    }
    if (motionStatus & 0x20) {
        armJointMoving[5] = true;
    }

    ::memcpy(armJointPos, &buffer[4], 6);
    ::memset(armJointPosRads, 0, 6 * sizeof(double));
}

void SIP::ParseArmInfo(unsigned char* buffer) {
    int length = static_cast<int>(buffer[0]) - 2;
    if (buffer[1] != ARMINFOPAC) {
#ifdef P2OS_ERROR_PRINT
        this->debug_serial->printf("error: Attempt to parse a non ARMINFO packet as arm info.\n");
#endif
        return;
    }

    if (length < 1) {
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->printf("debug: ARMINFOpac length bad size\n");
#endif
        return;
    }

    // Copy the version string
    // if (armVersionString != "") {
    free(armVersionString);
    // }

    // strndup() isn't available everywhere (e.g., Darwin)
    // armVersionString = strndup ((char*) &buffer[2], length);  // Can't be any bigger than length
    armVersionString = reinterpret_cast<char*>(calloc(length + 1, sizeof(char)));
    assert(armVersionString);
    strncpy(armVersionString, reinterpret_cast<char*>(&buffer[2]), length);
    int dataOffset = strlen(armVersionString) + 3;  // 1 for packet size byte, 1 ID, 1 for '\0'
    armNumJoints = buffer[dataOffset];
    if (armJoints) {
        delete[] armJoints;
    }
    if (armNumJoints <= 0) {
        return;
    }
    armJoints = new ArmJoint[armNumJoints];
    dataOffset += 1;
    for (int ii = 0; ii < armNumJoints; ii++) {
        armJoints[ii].speed = buffer[dataOffset + (ii * 6)];
        armJoints[ii].home = buffer[dataOffset + (ii * 6) + 1];
        armJoints[ii].min = buffer[dataOffset + (ii * 6) + 2];
        armJoints[ii].centre = buffer[dataOffset + (ii * 6) + 3];
        armJoints[ii].max = buffer[dataOffset + (ii * 6) + 4];
        armJoints[ii].ticksPer90 = buffer[dataOffset + (ii * 6) + 5];
    }
}
