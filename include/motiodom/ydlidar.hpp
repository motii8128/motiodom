#ifndef MOTISLAM_YDLIDAR_HPP_
#define MOTISLAM_YDLIDAR_HPP_

#include "types.hpp"

#include <CYdLidar.h>
#include <core/common/ydlidar_help.h>
#include <string>
#include <map>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace motiodom
{
    class YDLidarDriver
    {
        public:
        /// @brief コンストラクタ
        /// @param baudrate シリアル通信速度
        YDLidarDriver(int baudrate=230400);

        /// @brief セットアップ
        /// @return 実行結果（trueなら成功、falseなら失敗）
        bool startLidar();

        /// @brief Lidarをシャットダウンする
        /// @return 返り値なし
        void closeLidar();

        /// @brief エラー文を取得するパッケージ
        /// @return エラー文の文字列
        const char* getError();

        /// @brief スキャン範囲の角度を取得する
        /// @return 角度[rad]
        float getPitchAngle();

        /// @brief スキャンを実行する
        /// @return 実行結果（trueなら成功、falseなら失敗）
        bool Scan();

        /// @brief 点群を取得する
        /// @return ２次元点群
        PointCloud2d getScanPoints();

        private:
        std::unique_ptr<CYdLidar> lidar_;
        std::string port_;
        int baudrate_;
        float pitch_;
        std::unique_ptr<LaserScan> scan_;
    };
}

#endif