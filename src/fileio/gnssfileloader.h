/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GNSSFILELOADER_H
#define GNSSFILELOADER_H

#include "common/angle.h"
#include "common/types.h"
#include "fileloader.h"

class GnssFileLoader : public FileLoader {

public:
    GnssFileLoader() = delete;
    explicit GnssFileLoader(const string &filename, int columns = 7) {
        open(filename, columns, FileLoader::TEXT);
    }

    const GNSS &next() {
        data_ = load();

        gnss_.time = data_[0];
        memcpy(gnss_.blh.data(), &data_[1], 3 * sizeof(double));
        gnss_.blh[0] *= D2R;
        gnss_.blh[1] *= D2R;

        gnss_.hasvelocity = false;
        gnss_.vel.setZero();
        gnss_.vel_std.setZero();

        // 13列GNSS文件格式: time(1)+blh(3)+vel_NED(3)+pos_std(3)+vel_std(3)
        // 7列GNSS文件格式:  time(1)+blh(3)+pos_std(3)
        // 13-column GNSS file: time(1)+blh(3)+vel_NED(3)+pos_std(3)+vel_std(3)
        // 7-column  GNSS file: time(1)+blh(3)+pos_std(3)
        if (data_.size() >= 13) {
            memcpy(gnss_.vel.data(), &data_[4], 3 * sizeof(double));
            memcpy(gnss_.std.data(), &data_[7], 3 * sizeof(double));
            memcpy(gnss_.vel_std.data(), &data_[10], 3 * sizeof(double));
            gnss_.hasvelocity = true;
        } else {
            memcpy(gnss_.std.data(), &data_[4], 3 * sizeof(double));
        }

        return gnss_;
    }

private:
    GNSS gnss_;
    vector<double> data_;
};

#endif // GNSSFILELOADER_H
