/*
 * パーティクルフィルタを用いて自己位置推定を行いながら経路追従を行う
 *
 * 開発者：赤井直紀
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <random>

// 構造体
typedef struct {
    float x, y;
} Point;

// パーティクルの構造体
typedef struct {
    float x, y, t; // 状態
    double w; // 重み
} Particle;

int main(int argc, char **argv) {
    // 描画をするためのファイル
    FILE *gp;
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set colors classic\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set size ratio 1 1\n");
    fprintf(gp, "set xlabel X [m]\n");
    fprintf(gp, "set ylabel Y [m]\n");
    fprintf(gp, "set tics font \"Arial, 14\"\n");
    fprintf(gp, "set xlabel font \"Arial, 14\"\n");
    fprintf(gp, "set ylabel font \"Arial, 14\"\n");

    // 経路の作成
    std::vector<Point> path; // 経路データを持つベクトル
    FILE *fp = fopen("path.txt", "w"); // 描画用のファイル

    Point p;
    p.x = 0.0f, p.y = 0.0f;
    path.push_back(p);
    fprintf(fp, "%f %f\n", p.x, p.y);

    p.x = 3.0f, p.y = 0.0f;
    path.push_back(p);
    fprintf(fp, "%f %f\n", p.x, p.y);

    p.x = 3.0f, p.y = 3.0f;
    path.push_back(p);
    fprintf(fp, "%f %f\n", p.x, p.y);

    p.x = 6.0f, p.y = 3.0f;
    path.push_back(p);
    fprintf(fp, "%f %f\n", p.x, p.y);

    p.x = 6.0f, p.y = 6.0f;
    path.push_back(p);
    fprintf(fp, "%f %f\n", p.x, p.y);

    fclose(fp);

     // ロボットの初期位置
    float roboX = 0.0f; // m
    float roboY = 0.0f; // m
    float roboT = 0.0f; // -pi ~ pi

    // 制御パラメータ
    // **************************************************
    float vmax = 1.0f; // 最大速度 [m/s]
    float Kvet = 0.2f, Kvel = 0.2f; // 偏差に対する減速の係数

    float lookaheadDist = 0.5f; // lookahead distance [m]

    // PID制御のゲイン
    float Kpel = 3.0f, Kdel = 2.0f, Kiel = 0.0f;
    float Kpet = 3.0f, Kdet = 2.0f, Kiet = 0.0f;
    // **************************************************

    // PID制御計算のための変数
    float prevEl = 0.0f, prevEt = 0.0f;
    float elSum = 0.0f, etSum = 0.0f;

    // タイムステップ
    float dt = 0.1f; // sec

    // 経路追従のための変数
    int waypoint = 0;
    int lastWaypoint = (int)path.size() - 1;

    // ロボットの真値に関するデータ
    float gtRoboX = roboX; // m
    float gtRoboY = roboY; // m
    float gtRoboT = roboT; // -pi ~ pi

    // 乱数に関するパラメータ
    std::random_device seed;
    std::default_random_engine engine(seed());

    // GNSSの観測に対するノイズ（GNSSからyaw角も取得できると仮定）
    std::normal_distribution<> gnssXDist(0.0, 0.2);
    std::normal_distribution<> gnssYDist(0.0, 0.2);
    std::normal_distribution<> gnssTDist(0.0, 0.2);

    // 移動量に関するノイズ
    std::normal_distribution<> vDist(0.0, 0.5);
    std::normal_distribution<> wDist(0.0, 0.5);

    // 初期位置に関するノイズ
    std::normal_distribution<> xInitDist(0.0, 0.2);
    std::normal_distribution<> yInitDist(0.0, 0.2);
    std::normal_distribution<> tInitDist(0.0, 0.2);

    // リサンプリングのための乱数
    std::uniform_real_distribution<double> uDist(0, 1);

    // falseの場合パーティクルフィルタによる推定を使わない
    // 推定を行わない場合誤差が蓄積して増大することを確認するために利用
    bool usePFEst = true;

    // パーティクルの定義と初期化
    int particleNum = 100;
    std::vector<Particle> particles(particleNum);
    for (int i = 0; i < particleNum; ++i) {
        float x = roboX + xInitDist(engine);
        float y = roboY + yInitDist(engine);
        float t = roboT + tInitDist(engine);
        while (t < -M_PI)
            t += 2.0f * M_PI;
        while (t > M_PI)
            t -= 2.0f * M_PI;
        particles[i].x = x;
        particles[i].y = y;        
        particles[i].t = t;
        particles[i].w = 1.0 / (double)particleNum;
    }

    // 動作モデルを用いたパーティクルの更新用に使用する変数
    float prevV = 0.0f, prevW = 0.0f;

    // 真値更新の際に制御入力に加える誤差の比率
    float lvelNoiseRate = 0.05f;
    float avelNoiseRate = -0.05f;

    // シミュレーション結果を書き込むファイル
    FILE *logFP = fopen("sim_result.txt", "w");
    int cnt = 0;

    // 最終地点に到着するまでループする
    while (waypoint < lastWaypoint) {
        // GNSSの観測を取得
        float gnssX = gtRoboX + gnssXDist(engine);
        float gnssY = gtRoboY + gnssYDist(engine);
        float gnssT = gtRoboT + gnssTDist(engine);
        while (gnssT < -M_PI)
            gnssT += 2.0f * M_PI;
        while (gnssT > M_PI)
            gnssT -= 2.0f * M_PI;

        // パーティクルの位置の更新かつ観測モデルを用いた尤度計算
        double sum = 0.0f; // 尤度の総和
        for (int i = 0; i < particleNum; ++i) {
            float pV = prevV + vDist(engine);
            float pW = prevW + wDist(engine);
            float pX = particles[i].x + pV * dt * cos(particles[i].t);
            float pY = particles[i].y + pV * dt * sin(particles[i].t);
            float pT = particles[i].t + pW * dt;
            while (pT < -M_PI)
                pT += 2.0f * M_PI;
            while (pT > M_PI)
                pT -= 2.0f * M_PI;
            particles[i].x = pX;
            particles[i].y = pY;
            particles[i].t = pT;

            float dx = pX - gnssX;
            float dy = pY - gnssY;
            float dt = pT - gnssT;
            while (dt < -M_PI)
                dt += 2.0f * M_PI;
            while (dt > M_PI)
                dt -= 2.0f * M_PI;
            double dist = dx * dx / 0.2 + dy * dy / 0.2 + dt * dt / 0.2;
            double w = exp(-dist * dist);
            particles[i].w = w;
            sum += w;
        }

        // 重み付き平均の計算
        float xSum = 0.0f, ySum = 0.0f, tSum = 0.0f;
        std::vector<double> wBuffer(particleNum);
        for (int i = 0; i < particleNum; ++i) {
            // 重みの正規化
            double w = particles[i].w / sum;
            xSum += particles[i].x * w;
            ySum += particles[i].y * w;
            float dt = roboT - particles[i].t;
            while (dt < -M_PI)
                dt += 2.0f * M_PI;
            while (dt > M_PI)
                dt -= 2.0f * M_PI;
            tSum += dt * w;
            particles[i].w = w;
            if (i == 0)
                wBuffer[i] = w;
            else
                wBuffer[i] = w + wBuffer[i - 1];
        }

        // パーティクルフィルタによる推定を行う場合は推定位置を重み付き平均で更新
        if (usePFEst) {
            roboX = xSum;
            roboY = ySum;
            roboT -= tSum;
            while (roboT < -M_PI)
                roboT += 2.0f * M_PI;
            while (roboT > M_PI)
                roboT -= 2.0f * M_PI;
        }

       // パーティクルのリサンプリング（不要なパーティクルの削除）
        double wo = 1.0 / (double)particleNum;
        std::vector<Particle> tmpParticles = particles;
        for (int i = 0; i < particleNum; ++i) {
            double darts = uDist(engine);
            for (int j = 0; j < particleNum; ++j) {
                if (darts < wBuffer[j]) {
                    particles[i].x = tmpParticles[j].x;
                    particles[i].y = tmpParticles[j].y;
                    particles[i].t = tmpParticles[j].t;
                    particles[i].w = wo;
                    break;
                }
            }
        }        

        // ロボットと経路までの横方向の距離の誤差の定義
        float pdx = path[waypoint + 1].x - path[waypoint].x;
        float pdy = path[waypoint + 1].y - path[waypoint].y;
        float pTheta = atan2(pdy, pdx);
        float rdx = roboX - path[waypoint].x;
        float rdy = roboY - path[waypoint].y;
        // float xx = rdx * cos(-pTheta) - rdy * sin(-pTheta);
        float yy = rdx * sin(-pTheta) + rdy * cos(-pTheta);
        float el = -yy;

        // 角度誤差の定義
        float et = pTheta - roboT;
        while (et < -M_PI)
            et += 2.0f * M_PI;
        while (et > M_PI)
            et -= 2.0f * M_PI;

        // 制御入力の決定
        float del = el - prevEl;
        float det = et - prevEt;
        while (det < -M_PI)
            det += 2.0f * M_PI;
        while (det > M_PI)
            det -= 2.0f * M_PI;

        elSum += el;
        etSum += et;

        float v = vmax - Kvel * fabs(el) - Kvet * fabs(et);
        float w = Kpet * et + Kdet * det + Kiet * etSum;
        w += Kpel * el + Kdel * del + Kiel * elSum;

        // 微分項計算のために前回の誤差を更新
        prevEl = el;
        prevEt = et;

        prevV = v;
        prevW = w;

        // ロボット推定位置の更新
        float x = roboX + v * dt * cos(roboT);
        float y = roboY + v * dt * sin(roboT);
        float th = roboT + w * dt;
        while (th < -M_PI)
            th += 2.0f * M_PI;
        while (th > M_PI)
            th -= 2.0f * M_PI;

        // ロボット位置の真値の更新（並進、角速度に一定量のノイズを加える）
        float gtV = v * (1.0f + lvelNoiseRate);
        float gtW = w * (1.0f + avelNoiseRate);
        float gtX = gtRoboX + gtV * dt * cos(gtRoboT);
        float gtY = gtRoboY + gtV * dt * sin(gtRoboT);
        float gtTh = gtRoboT + gtW * dt;
        while (gtTh < -M_PI)
            gtTh += 2.0f * M_PI;
        while (gtTh > M_PI)
            gtTh -= 2.0f * M_PI;

        // 推定位置でのロボットボディをファイルに記録
        FILE *fp = fopen("body.txt", "w");
        float x1 = 1.0f;
        float y1 = 0.0f;
        float x2 = -0.5f;
        float y2 = 0.5f;
        float x3 = -0.5f;
        float y3 = -0.5f;

        float x11 = x1 * cos(th) - y1 * sin(th) + x;
        float y11 = x1 * sin(th) + y1 * cos(th) + y;
        float x22 = x2 * cos(th) - y2 * sin(th) + x;
        float y22 = x2 * sin(th) + y2 * cos(th) + y;
        float x33 = x3 * cos(th) - y3 * sin(th) + x;
        float y33 = x3 * sin(th) + y3 * cos(th) + y;

        fprintf(fp, "%f %f\n", x11, y11);
        fprintf(fp, "%f %f\n", x22, y22);
        fprintf(fp, "%f %f\n", x33, y33);
        fprintf(fp, "%f %f\n", x11, y11);
        fclose(fp);

        // 真値位置でのロボットボディをファイルに記録
        fp = fopen("gt_body.txt", "w");
        x1 = 1.0f;
        y1 = 0.0f;
        x2 = -0.5f;
        y2 = 0.5f;
        x3 = -0.5f;
        y3 = -0.5f;

        x11 = x1 * cos(gtTh) - y1 * sin(gtTh) + gtX;
        y11 = x1 * sin(gtTh) + y1 * cos(gtTh) + gtY;
        x22 = x2 * cos(gtTh) - y2 * sin(gtTh) + gtX;
        y22 = x2 * sin(gtTh) + y2 * cos(gtTh) + gtY;
        x33 = x3 * cos(gtTh) - y3 * sin(gtTh) + gtX;
        y33 = x3 * sin(gtTh) + y3 * cos(gtTh) + gtY;

        fprintf(fp, "%f %f\n", x11, y11);
        fprintf(fp, "%f %f\n", x22, y22);
        fprintf(fp, "%f %f\n", x33, y33);
        fprintf(fp, "%f %f\n", x11, y11);
        fclose(fp);

        fp = fopen("gnss.txt", "w");
        fprintf(fp, "%f %f %f\n", gnssX, gnssY, gnssT);
        fclose(fp);

        fp = fopen("particles.txt", "w");
        for (int i = 0; i < particleNum; ++i)
            fprintf(fp, "%f %f %f\n", particles[i].x, particles[i].y, particles[i].t);
        fclose(fp);

        // 推定位置を中心に描画
        fprintf(gp, "set xrange [%f : %f]\n", x - 4.0f, x + 4.0f);
        fprintf(gp, "set yrange [%f : %f]\n", y - 4.0f, y + 4.0f);
        fprintf(gp, "plot \"path.txt\" lt -1 w l t \"Path\", "
            "\"particles.txt\" t \"Particles\", "
            "\"body.txt\" w l lt 3 t \"Estimate\", "
            "\"gt_body.txt\" w l lt 1 t \"Robot\", "
            "\"gnss.txt\" lw 4 lt 7 t \"GNSS\"\n");
        fflush(gp);

        // シミュレーション結果の記録
        fprintf(logFP, "%d %f %f %f %f %f %f %f %f %f\n",
            cnt,
            gtRoboX, gtRoboY, gtRoboT,
            roboX, roboY, roboT,
            gnssX, gnssY, gnssT);

        // 次のループのためにロボット位置を保存
        roboX = x;
        roboY = y;
        roboT = th;

        gtRoboX = gtX;
        gtRoboY = gtY;
        gtRoboT = gtTh;

        // 経路waypointの更新
        float rdx2 = path[waypoint + 1].x - roboX;
        float rdy2 = path[waypoint + 1].y - roboY;
        float rdl2 = sqrt(rdx2 * rdx2 + rdy2 * rdy2);
        if (rdl2 <= lookaheadDist)
            waypoint++;

        usleep(dt * 10e5);
        cnt++;
    }

    fclose(gp);

    int retVal = system("killall -9 gnuplot\n");

    return 0;
}

