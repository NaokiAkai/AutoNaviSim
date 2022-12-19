/*
 * ロボットのボディを描画するためだけの確認用ソフト
 *
 * 開発者：赤井直紀
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>

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

    // 10秒間表示
    float dt = 0.1f;
    for (float t = 0.0; t < 10.0f; t += dt) {
        // ロボット位置を時間により更新
        float x = cos(t);
        float y = sin(t);
        float th = M_PI / 2.0 + atan2(y, x);

        // ロボットボディの記録
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

        // 描画
        fprintf(gp, "set xrange [%f : %f]\n", x - 4.0f, x + 4.0f);
        fprintf(gp, "set yrange [%f : %f]\n", y - 4.0f, y + 4.0f);
        fprintf(gp, "plot \"body.txt\" lt 1 w l t \"Robot\"\n");
        fflush(gp);

        usleep(dt * 10e5);
    }

    fclose(gp);

    int retVal = system("killall -9 gnuplot\n");

    return 0;
}

