/*
 * 円軌道の経路追従を行う
 *
 * 開発者：赤井直紀
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

// 構造体
typedef struct {
    float x, y;
} Point;

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
    // 経路作成終了

    // 制御パラメータ
    // **************************************************
     // ロボットの初期位置
    float roboX = 0.0f; // m
    float roboY = 0.0f; // m
    float roboT = 0.0f; // -pi ~ pi

    float vmax = 1.0f; // 最大速度
    float Kvet = 1.0f, Kvel = 1.0f; // 偏差に対する減速の係数

    float lookaheadDist = 0.05f; // m

    // PID制御のゲイン
    float Kpel = 3.0f, Kdel = 0.0f, Kiel = 0.0f;
    float Kpet = 3.0f, Kdet = 0.0f, Kiet = 0.0f;

    // PID制御計算のための変数
    float prevEl = 0.0f, prevEt = 0.0f;
    float elSum = 0.0f, etSum = 0.0f;
    // **************************************************

    // タイムステップ
    float dt = 0.1f; // sec

    // 経路追従のための変数
    int waypoint = 0;
    int lastWaypoint = (int)path.size() - 1;

    // 最終目的地に到着するまでループを繰り返す
    while (waypoint < lastWaypoint) {
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

        // ロボット位置の更新
        float x = roboX + v * dt * cos(roboT);
        float y = roboY + v * dt * sin(roboT);
        float th = roboT + w * dt;
        while (th < -M_PI)
            th += 2.0f * M_PI;
        while (th > M_PI)
            th -= 2.0f * M_PI;

        fp = fopen("body.txt", "w");
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

        // 描画
        fprintf(gp, "set xrange [%f : %f]\n", x - 4.0f, x + 4.0f);
        fprintf(gp, "set yrange [%f : %f]\n", y - 4.0f, y + 4.0f);
        fprintf(gp, "plot \"path.txt\" lt -1 w l t \"Path\", "
            "\"body.txt\" w l lt 1 t \"Robot\"\n");
        fflush(gp);

        // 次のループのためにロボット位置を保存
        roboX = x;
        roboY = y;
        roboT = th;

        // waypointの更新
        float rdx2 = path[waypoint + 1].x - roboX;
        float rdy2 = path[waypoint + 1].y - roboY;
        float rdl2 = sqrt(rdx2 * rdx2 + rdy2 * rdy2);
        if (rdl2 <= lookaheadDist)
            waypoint++;

        usleep(dt * 10e5);
    }

    fclose(gp);

    int retVal = system("killall -9 gnuplot\n");

    return 0;
}

