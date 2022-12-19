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
    float R = 3.0f; // 円の半径
    std::vector<Point> path; // 経路データを持つベクトル
    FILE *fp = fopen("path.txt", "w"); // 描画用のファイル
    for (float t = 0.0f; t < 2.0f * M_PI; t += 0.1f) {
        Point p;
        p.x = R * cos(t);
        p.y = R * sin(t);
        path.push_back(p); // ベクトルにデータを追加
         // 描画用にファイルに書き込み 
        fprintf(fp, "%f %f\n", p.x, p.y);
    }
    fprintf(fp, "%f %f\n", path[0].x, path[0].y);
    fclose(fp);

     // ロボットの初期位置
    float roboX = R; // m
    float roboY = 0.0f; // m
    float roboT = M_PI / 2.0f; // -pi ~ pi

    // 制御パラメータ
    // **************************************************
    float vmax = 1.0f; // 最大速度 [m/s]
    float Kvet = 0.1f, Kvel = 0.1f; // 偏差に対する減速の係数

    float lookaheadDist = 1.0f; // lookahead distance [m]

    // PID制御のゲイン（左からP、I、Dのゲイン。上がel、下がetのゲイン）
    float Kpel = 3.0f, Kdel = 0.0f, Kiel = 0.0f;
    float Kpet = 3.0f, Kdet = 0.0f, Kiet = 0.0f;
    // **************************************************

    // PID制御の計算のために使用される変数
    float prevEl = 0.0f, prevEt = 0.0f;
    float elSum = 0.0f, etSum = 0.0f;

    // タイムステップ
    float dt = 0.1f; // sec

    // 10秒間走行
    for (float t = 0.0; t < 10.0f; t += dt) {
        // ロボットと円軌道の距離誤差の定義
        float el;

        // 角度誤差の定義
        float et;

        // 積分項計算のための総和を計算
        elSum += el;
        etSum += et;

        // PID制御を用いた制御入力の計算
        float v;
        float w;

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

        // ロボットボディの記録
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
            "\"body.txt\" lt 1 w l t \"Robot\"\n");
        fflush(gp);

        // 次のループのためにロボット位置を保存
        roboX = x;
        roboY = y;
        roboT = th;

        usleep(dt * 10e5);
    }

    fclose(gp);

    int retVal = system("killall -9 gnuplot\n");

    return 0;
}

