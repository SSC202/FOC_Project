#include "my_math.h"

/**
 * @brief   机械角度转换为电角度
 * @param   polepairs       电机极对数(带方向,>0表示电角度正方向和编码器机械角度正方向相同)
 * @param   mechine_angle   电机机械角度(编码器返回角度)
 * @param   offset          电机电角度零点和电机机械角度零点之间的偏差
 *                          该偏差由下述方法整定:
 *                          设定一个ud值,uq设置为0,启动电机,电机将会旋转到电角度零点,此时读取编码器角度即为偏差
 * @return  电机的电角度,归一化为 -PI 到 PI 之间
 */
float normalize(int pole_pairs, float mechine_angle, float offset)
{
    float electric_angle = pole_pairs * mechine_angle - offset;
    float out            = electric_angle;
    while (out < -M_PI) {
        out = out + 2 * M_PI;
    }
    while (out > M_PI) {
        out = out - 2 * M_PI;
    }
    return out;
}

/**
 * @brief   取最大值函数
 */
float get_max(float a, float b, float c)
{
    float ret = 0;
    if (a >= b && a >= c)
        ret = a;
    else if (b >= a && b >= c)
        ret = b;
    else
        ret = c;
    return ret;
}

/**
 * @brief   取最小值函数
 */
float get_min(float a, float b, float c)
{
    float ret = 0;
    if (a <= b && a <= c)
        ret = a;
    else if (b <= a && b <= c)
        ret = b;
    else
        ret = c;
    return ret;
}

/**
 * @brief   取中间值函数
 */
float get_middle(float a, float b, float c)
{
    float ret = 0;
    ret       = a + b + c - get_max(a, b, c) - get_min(a, b, c);
    return ret;
}

/****************************** PID 控制器 ********************************* */

/**
 * @brief   增量式 PID 控制器初始化
 * @param   pid         PID 控制器句柄
 * @param   kp          Kp  比例参数
 * @param   ki          Ki  积分参数(考虑采样时间)
 * @param   kd          Kd  微分参数
 * @param   outputMax   输出饱和值
 */
void PID_init(PID_t *pid, float kp, float ki, float kd, float outputMax)
{
    pid->KP        = kp;
    pid->KI        = ki;
    pid->KD        = kd;
    pid->outputMax = outputMax;
}

/**
 * @brief   增量式PID计算
 * @param   pid         PID 控制器句柄
 * @param   enable      使能信号    若此值为0,PID控制器的输入,反馈和输出都置为0
 * @param   t_sample    采样时间
 */
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample)
{
    pid->ref       = pid->ref * enable;
    pid->fdb       = pid->fdb * enable;
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * t_sample * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->output   = pid->output * enable;
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    /*设定输出上限*/
    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
}

/****************************** IIR 滤波器 ********************************* */

/**
 * @brief   数字一阶低通滤波器初始化(后向差分法)
 * @param   lpf         滤波器句柄
 * @param   f_c         截止频率
 * @param   t_sample    采样时间
 */
void LPF_Init(LPF_t *lpf, float f_c, float t_sample)
{
    lpf->wc          = 2 * M_PI * f_c;
    lpf->tsample     = t_sample;
    lpf->alpha       = (lpf->wc * lpf->tsample) / ((lpf->wc * lpf->tsample) + 1.0f);
    lpf->output_last = 0;
    lpf->output      = 0;
    lpf->input       = 0;
}

/**
 * @brief   数字一阶低通滤波器计算(后向差分法)
 * @param   lpf         滤波器句柄
 */
void LPF_Calc(LPF_t *lpf)
{
    lpf->output      = lpf->output_last + lpf->alpha * (lpf->input - lpf->output_last);
    lpf->output_last = lpf->output;
}

/****************************** SIN ********************************* */
// 正弦查找表
float sin_table[M_TABLE_SIZE] = {
    0.000000, 0.006142, 0.012284, 0.018425,
    0.024565, 0.030705, 0.036843, 0.042980,
    0.049116, 0.055249, 0.061381, 0.067510,
    0.073636, 0.079760, 0.085881, 0.091999,
    0.098113, 0.104223, 0.110330, 0.116432,
    0.122530, 0.128623, 0.134711, 0.140795,
    0.146873, 0.152945, 0.159012, 0.165073,
    0.171127, 0.177175, 0.183217, 0.189251,
    0.195279, 0.201299, 0.207311, 0.213315,
    0.219312, 0.225300, 0.231280, 0.237251,
    0.243213, 0.249166, 0.255109, 0.261043,
    0.266967, 0.272881, 0.278785, 0.284678,
    0.290560, 0.296432, 0.302292, 0.308141,
    0.313978, 0.319803, 0.325617, 0.331418,
    0.337206, 0.342982, 0.348745, 0.354494,
    0.360231, 0.365954, 0.371662, 0.377357,
    0.383038, 0.388704, 0.394356, 0.399993,
    0.405614, 0.411220, 0.416811, 0.422386,
    0.427945, 0.433488, 0.439015, 0.444525,
    0.450018, 0.455495, 0.460954, 0.466396,
    0.471820, 0.477226, 0.482615, 0.487985,
    0.493337, 0.498670, 0.503984, 0.509279,
    0.514555, 0.519812, 0.525049, 0.530266,
    0.535464, 0.540641, 0.545798, 0.550934,
    0.556049, 0.561143, 0.566216, 0.571268,
    0.576298, 0.581307, 0.586294, 0.591258,
    0.596200, 0.601120, 0.606017, 0.610891,
    0.615742, 0.620570, 0.625374, 0.630155,
    0.634912, 0.639646, 0.644355, 0.649039,
    0.653700, 0.658335, 0.662946, 0.667532,
    0.672092, 0.676627, 0.681137, 0.685621,
    0.690079, 0.694511, 0.698917, 0.703296,
    0.707649, 0.711976, 0.716275, 0.720548,
    0.724793, 0.729011, 0.733201, 0.737364,
    0.741499, 0.745606, 0.749684, 0.753735,
    0.757757, 0.761751, 0.765715, 0.769651,
    0.773558, 0.777436, 0.781284, 0.785103,
    0.788892, 0.792652, 0.796381, 0.800081,
    0.803750, 0.807389, 0.810998, 0.814576,
    0.818123, 0.821640, 0.825125, 0.828579,
    0.832002, 0.835394, 0.838754, 0.842083,
    0.845379, 0.848644, 0.851877, 0.855078,
    0.858246, 0.861382, 0.864486, 0.867557,
    0.870595, 0.873600, 0.876573, 0.879512,
    0.882418, 0.885291, 0.888131, 0.890937,
    0.893709, 0.896448, 0.899153, 0.901824,
    0.904461, 0.907063, 0.909632, 0.912166,
    0.914666, 0.917131, 0.919562, 0.921958,
    0.924320, 0.926646, 0.928938, 0.931194,
    0.933415, 0.935601, 0.937752, 0.939868,
    0.941948, 0.943992, 0.946001, 0.947974,
    0.949911, 0.951813, 0.953678, 0.955508,
    0.957302, 0.959059, 0.960781, 0.962466,
    0.964114, 0.965727, 0.967303, 0.968842,
    0.970345, 0.971812, 0.973241, 0.974634,
    0.975990, 0.977310, 0.978592, 0.979838,
    0.981047, 0.982218, 0.983353, 0.984450,
    0.985511, 0.986534, 0.987520, 0.988468,
    0.989380, 0.990254, 0.991091, 0.991890,
    0.992652, 0.993376, 0.994063, 0.994713,
    0.995325, 0.995899, 0.996436, 0.996935,
    0.997397, 0.997821, 0.998208, 0.998556,
    0.998867, 0.999141, 0.999376, 0.999574,
    0.999735, 0.999857, 0.999942, 0.999989,
    0.999999, 0.999971, 0.999905, 0.999801,
    0.999659, 0.999480, 0.999263, 0.999009,
    0.998717, 0.998387, 0.998019, 0.997614,
    0.997171, 0.996690, 0.996172, 0.995617,
    0.995023, 0.994393, 0.993724, 0.993019,
    0.992276, 0.991495, 0.990677, 0.989821,
    0.988929, 0.987999, 0.987031, 0.986027,
    0.984985, 0.983906, 0.982790, 0.981637,
    0.980447, 0.979220, 0.977956, 0.976655,
    0.975317, 0.973942, 0.972531, 0.971083,
    0.969598, 0.968077, 0.966519, 0.964925,
    0.963295, 0.961628, 0.959924, 0.958185,
    0.956409, 0.954598, 0.952750, 0.950867,
    0.948947, 0.946992, 0.945001, 0.942974,
    0.940912, 0.938814, 0.936681, 0.934513,
    0.932309, 0.930070, 0.927796, 0.925487,
    0.923143, 0.920765, 0.918351, 0.915903,
    0.913421, 0.910903, 0.908352, 0.905766,
    0.903146, 0.900492, 0.897805, 0.895083,
    0.892327, 0.889538, 0.886715, 0.883859,
    0.880969, 0.878047, 0.875091, 0.872102,
    0.869080, 0.866025, 0.862938, 0.859818,
    0.856666, 0.853481, 0.850265, 0.847016,
    0.843735, 0.840422, 0.837078, 0.833702,
    0.830295, 0.826856, 0.823386, 0.819885,
    0.816353, 0.812791, 0.809197, 0.805574,
    0.801919, 0.798235, 0.794520, 0.790776,
    0.787001, 0.783197, 0.779364, 0.775501,
    0.771608, 0.767687, 0.763737, 0.759757,
    0.755750, 0.751713, 0.747649, 0.743556,
    0.739435, 0.735286, 0.731109, 0.726905,
    0.722674, 0.718415, 0.714129, 0.709816,
    0.705476, 0.701110, 0.696717, 0.692298,
    0.687853, 0.683382, 0.678885, 0.674363,
    0.669815, 0.665242, 0.660644, 0.656020,
    0.651372, 0.646700, 0.642003, 0.637282,
    0.632537, 0.627768, 0.622975, 0.618159,
    0.613319, 0.608457, 0.603571, 0.598663,
    0.593732, 0.588779, 0.583803, 0.578805,
    0.573786, 0.568745, 0.563682, 0.558599,
    0.553494, 0.548368, 0.543222, 0.538055,
    0.532868, 0.527660, 0.522433, 0.517186,
    0.511920, 0.506634, 0.501329, 0.496005,
    0.490663, 0.485302, 0.479923, 0.474525,
    0.469110, 0.463677, 0.458227, 0.452759,
    0.447274, 0.441772, 0.436254, 0.430719,
    0.425168, 0.419601, 0.414018, 0.408419,
    0.402805, 0.397176, 0.391532, 0.385873,
    0.380200, 0.374512, 0.368810, 0.363094,
    0.357364, 0.351621, 0.345865, 0.340096,
    0.334313, 0.328519, 0.322711, 0.316892,
    0.311061, 0.305218, 0.299363, 0.293497,
    0.287620, 0.281733, 0.275834, 0.269925,
    0.264006, 0.258077, 0.252139, 0.246190,
    0.240233, 0.234266, 0.228291, 0.222307,
    0.216315, 0.210314, 0.204306, 0.198289,
    0.192266, 0.186235, 0.180197, 0.174152,
    0.168101, 0.162043, 0.155979, 0.149910,
    0.143835, 0.137754, 0.131668, 0.125577,
    0.119481, 0.113381, 0.107277, 0.101168,
    0.095056, 0.088940, 0.082821, 0.076699,
    0.070573, 0.064445, 0.058315, 0.052183,
    0.046048, 0.039912, 0.033774, 0.027635,
    0.021495, 0.015354, 0.009213, 0.003071,
    -0.003071, -0.009213, -0.015354, -0.021495,
    -0.027635, -0.033774, -0.039912, -0.046048,
    -0.052183, -0.058315, -0.064445, -0.070573,
    -0.076699, -0.082821, -0.088940, -0.095056,
    -0.101168, -0.107277, -0.113381, -0.119481,
    -0.125577, -0.131668, -0.137754, -0.143835,
    -0.149910, -0.155979, -0.162043, -0.168101,
    -0.174152, -0.180197, -0.186235, -0.192266,
    -0.198289, -0.204306, -0.210314, -0.216315,
    -0.222307, -0.228291, -0.234266, -0.240233,
    -0.246190, -0.252139, -0.258077, -0.264006,
    -0.269925, -0.275834, -0.281733, -0.287620,
    -0.293497, -0.299363, -0.305218, -0.311061,
    -0.316892, -0.322711, -0.328519, -0.334313,
    -0.340096, -0.345865, -0.351621, -0.357364,
    -0.363094, -0.368810, -0.374512, -0.380200,
    -0.385873, -0.391532, -0.397176, -0.402805,
    -0.408419, -0.414018, -0.419601, -0.425168,
    -0.430719, -0.436254, -0.441772, -0.447274,
    -0.452759, -0.458227, -0.463677, -0.469110,
    -0.474525, -0.479923, -0.485302, -0.490663,
    -0.496005, -0.501329, -0.506634, -0.511920,
    -0.517186, -0.522433, -0.527660, -0.532868,
    -0.538055, -0.543222, -0.548368, -0.553494,
    -0.558599, -0.563682, -0.568745, -0.573786,
    -0.578805, -0.583803, -0.588779, -0.593732,
    -0.598663, -0.603571, -0.608457, -0.613319,
    -0.618159, -0.622975, -0.627768, -0.632537,
    -0.637282, -0.642003, -0.646700, -0.651372,
    -0.656020, -0.660644, -0.665242, -0.669815,
    -0.674363, -0.678885, -0.683382, -0.687853,
    -0.692298, -0.696717, -0.701110, -0.705476,
    -0.709816, -0.714129, -0.718415, -0.722674,
    -0.726905, -0.731109, -0.735286, -0.739435,
    -0.743556, -0.747649, -0.751713, -0.755750,
    -0.759757, -0.763737, -0.767687, -0.771608,
    -0.775501, -0.779364, -0.783197, -0.787001,
    -0.790776, -0.794520, -0.798235, -0.801919,
    -0.805574, -0.809197, -0.812791, -0.816353,
    -0.819885, -0.823386, -0.826856, -0.830295,
    -0.833702, -0.837078, -0.840422, -0.843735,
    -0.847016, -0.850265, -0.853481, -0.856666,
    -0.859818, -0.862938, -0.866025, -0.869080,
    -0.872102, -0.875091, -0.878047, -0.880969,
    -0.883859, -0.886715, -0.889538, -0.892327,
    -0.895083, -0.897805, -0.900492, -0.903146,
    -0.905766, -0.908352, -0.910903, -0.913421,
    -0.915903, -0.918351, -0.920765, -0.923143,
    -0.925487, -0.927796, -0.930070, -0.932309,
    -0.934513, -0.936681, -0.938814, -0.940912,
    -0.942974, -0.945001, -0.946992, -0.948947,
    -0.950867, -0.952750, -0.954598, -0.956409,
    -0.958185, -0.959924, -0.961628, -0.963295,
    -0.964925, -0.966519, -0.968077, -0.969598,
    -0.971083, -0.972531, -0.973942, -0.975317,
    -0.976655, -0.977956, -0.979220, -0.980447,
    -0.981637, -0.982790, -0.983906, -0.984985,
    -0.986027, -0.987031, -0.987999, -0.988929,
    -0.989821, -0.990677, -0.991495, -0.992276,
    -0.993019, -0.993724, -0.994393, -0.995023,
    -0.995617, -0.996172, -0.996690, -0.997171,
    -0.997614, -0.998019, -0.998387, -0.998717,
    -0.999009, -0.999263, -0.999480, -0.999659,
    -0.999801, -0.999905, -0.999971, -0.999999,
    -0.999989, -0.999942, -0.999857, -0.999735,
    -0.999574, -0.999376, -0.999141, -0.998867,
    -0.998556, -0.998208, -0.997821, -0.997397,
    -0.996935, -0.996436, -0.995899, -0.995325,
    -0.994713, -0.994063, -0.993376, -0.992652,
    -0.991890, -0.991091, -0.990254, -0.989380,
    -0.988468, -0.987520, -0.986534, -0.985511,
    -0.984450, -0.983353, -0.982218, -0.981047,
    -0.979838, -0.978592, -0.977310, -0.975990,
    -0.974634, -0.973241, -0.971812, -0.970345,
    -0.968842, -0.967303, -0.965727, -0.964114,
    -0.962466, -0.960781, -0.959059, -0.957302,
    -0.955508, -0.953678, -0.951813, -0.949911,
    -0.947974, -0.946001, -0.943992, -0.941948,
    -0.939868, -0.937752, -0.935601, -0.933415,
    -0.931194, -0.928938, -0.926646, -0.924320,
    -0.921958, -0.919562, -0.917131, -0.914666,
    -0.912166, -0.909632, -0.907063, -0.904461,
    -0.901824, -0.899153, -0.896448, -0.893709,
    -0.890937, -0.888131, -0.885291, -0.882418,
    -0.879512, -0.876573, -0.873600, -0.870595,
    -0.867557, -0.864486, -0.861382, -0.858246,
    -0.855078, -0.851877, -0.848644, -0.845379,
    -0.842083, -0.838754, -0.835394, -0.832002,
    -0.828579, -0.825125, -0.821640, -0.818123,
    -0.814576, -0.810998, -0.807389, -0.803750,
    -0.800081, -0.796381, -0.792652, -0.788892,
    -0.785103, -0.781284, -0.777436, -0.773558,
    -0.769651, -0.765715, -0.761751, -0.757757,
    -0.753735, -0.749684, -0.745606, -0.741499,
    -0.737364, -0.733201, -0.729011, -0.724793,
    -0.720548, -0.716275, -0.711976, -0.707649,
    -0.703296, -0.698917, -0.694511, -0.690079,
    -0.685621, -0.681137, -0.676627, -0.672092,
    -0.667532, -0.662946, -0.658335, -0.653700,
    -0.649039, -0.644355, -0.639646, -0.634912,
    -0.630155, -0.625374, -0.620570, -0.615742,
    -0.610891, -0.606017, -0.601120, -0.596200,
    -0.591258, -0.586294, -0.581307, -0.576298,
    -0.571268, -0.566216, -0.561143, -0.556049,
    -0.550934, -0.545798, -0.540641, -0.535464,
    -0.530266, -0.525049, -0.519812, -0.514555,
    -0.509279, -0.503984, -0.498670, -0.493337,
    -0.487985, -0.482615, -0.477226, -0.471820,
    -0.466396, -0.460954, -0.455495, -0.450018,
    -0.444525, -0.439015, -0.433488, -0.427945,
    -0.422386, -0.416811, -0.411220, -0.405614,
    -0.399993, -0.394356, -0.388704, -0.383038,
    -0.377357, -0.371662, -0.365954, -0.360231,
    -0.354494, -0.348745, -0.342982, -0.337206,
    -0.331418, -0.325617, -0.319803, -0.313978,
    -0.308141, -0.302292, -0.296432, -0.290560,
    -0.284678, -0.278785, -0.272881, -0.266967,
    -0.261043, -0.255109, -0.249166, -0.243213,
    -0.237251, -0.231280, -0.225300, -0.219312,
    -0.213315, -0.207311, -0.201299, -0.195279,
    -0.189251, -0.183217, -0.177175, -0.171127,
    -0.165073, -0.159012, -0.152945, -0.146873,
    -0.140795, -0.134711, -0.128623, -0.122530,
    -0.116432, -0.110330, -0.104223, -0.098113,
    -0.091999, -0.085881, -0.079760, -0.073636,
    -0.067510, -0.061381, -0.055249, -0.049116,
    -0.042980, -0.036843, -0.030705, -0.024565,
    -0.018425, -0.012284, -0.006142, -0.000000};

float fast_sin(float x)
{
    while (x < 0) {
        x += 2 * M_PI;
    }
    while (x >= 2 * M_PI) {
        x -= 2 * M_PI;
    }

    int index        = (int)((x / (2 * M_PI)) * M_TABLE_SIZE);
    float fractional = ((x / (2 * M_PI)) * M_TABLE_SIZE) - index;

    float y0 = sin_table[index];
    float y1 = sin_table[(index + 1) % M_TABLE_SIZE];
    return y0 + fractional * (y1 - y0);
}

float fast_cos(float x)
{
    return (fast_sin(x + M_PI / 2));
}

float fast_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int i       = *(int *)&x;                 // get bits for floating VALUE
    i           = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x           = *(float *)&i;               // convert bits BACK to float
    x           = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    x           = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    x           = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy

    return 1 / x;
}