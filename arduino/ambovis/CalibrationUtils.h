//
// Created by Mirian Quinteros on 27/02/2022.
//

#ifndef CALIBRATIONUTILS_H
#define CALIBRATIONUTILS_H

#define CALIB_CYCLES  5

void calibrate(bool calibration_run, float verror_sum, byte vcorr_count, byte calib_cycle, float verror_sum_outcycle, float vzero) {
    if (calibration_run) {
        float verror = verror_sum / float(vcorr_count);

        vcorr_count = verror_sum = 0.;
        calib_cycle ++;
        verror_sum_outcycle += verror;
        if (calib_cycle >= CALIB_CYCLES ) {
            calibration_run = false;
            vzero = verror_sum_outcycle / float(CALIB_CYCLES);
        }
    } else {
        float verror = verror_sum / float(vcorr_count);
        vcorr_count = verror_sum = 0.;
    }
}

#endif //CALIBRATIONUTILS_H
