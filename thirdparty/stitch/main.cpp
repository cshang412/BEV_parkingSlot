#include <iostream>
#include "stitch_interface.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <fstream>

static uchar clamp(float d) {
    if (d < 0.0f)
        return (uchar)0;
    if (d > 255.0f)
        return (uchar)255;
    return uchar(d);
}

static void RGB2YUV(uchar R, uchar G, uchar B, uchar *Y, uchar *U, uchar *V) {
    float a = 0.299 * R + 0.587 * G + 0.114 * B;
    float c = (R - a) * 0.713 + 128;
    float b = (B - a) * 0.564 + 128;
    // float b = -0.14713    * R - 0.28886   * G + 0.436    * B ;
    // float c =  0.615    * R - 0.51499    * G - 0.10001    * B;
    //*Y = (77*R + 150*G + 29*B)>>8;
    //*U = ((-44*R  - 87*G  + 131*B)>>8) + 128;
    //*V = ((131*R - 110*G - 21*B)>>8) + 128 ;
    *Y = clamp(a);
    *U = clamp(b);
    *V = clamp(c);
}

void convertToUYVY(cv::Mat &src, cv::Mat &yuyv) {
    yuyv = cv::Mat::zeros(src.rows, src.cols, CV_8UC2);
    uchar *src_data = src.data;
    uchar *dst_data = yuyv.data;
    int size = src.rows * src.cols;
    for(int i = 0; i < size; i+=2)
    {
        uchar r1 = src_data[i*3+2];
        uchar g1 = src_data[i*3+1];
        uchar b1 = src_data[i*3+0];

        uchar r2 = src_data[i*3+5];
        uchar g2 = src_data[i*3+4];
        uchar b2 = src_data[i*3+3];

        uchar y1, u1, v1, y2, u2, v2;
        RGB2YUV(r1, g1, b1, &y1, &u1, &v1);

        RGB2YUV(r2, g2, b2, &y2, &u2, &v2);

        uchar u = clamp((static_cast<float>(u1)+static_cast<float>(u2))/2);
        uchar v = clamp((static_cast<float>(v1)+static_cast<float>(v2))/2);

        dst_data[i*2+0] = u;
        dst_data[i*2+1] = y1;
        dst_data[i*2+2] = v;
        dst_data[i*2+3] = y2;
    }
//    uchar  R, G, B, Y, U, V;
//    int    count = 0;
//    for (int r = 0; r < src.rows; r++) {
//        for (int c = 0; c < src.cols; c++) {
//            cv::Vec3b color = src.at<cv::Vec3b>(r, c);
//            R = color[2];
//            G = color[1];
//            B = color[0];
//
//            RGB2YUV(R, G, B, &Y, &U, &V);
//
//            data[count * 2 + 1] = Y;
//            if (count % 2 == 0) {
//                data[2 * count] = U;
//            } else {
//                data[2 * count] = V;
//            }
//            count++;
//        }
//    }
}
#include "json/json.h"

static Json::Value load_json(const std::string& file_name)
{

    std::ifstream ifile;
    ifile.open(file_name);

    Json::CharReaderBuilder ReaderBuilder;
    ReaderBuilder["emitUTF8"] = true;//utf8支持，不加这句，utf8的中文字符会编程\uxxx
    Json::Value value;

    std::string strerr;
    if(!Json::parseFromStream(ReaderBuilder,ifile,&value,&strerr))
    {
        std::cerr<<"load json map"<<file_name<<" failed!"<<std::endl;
        exit(0);
    }

    ifile.close();

    return value;
}

cv::Point2f parse(const Json::Value& value)
{
    cv::Point3d p(value["point"]["x"].asDouble(),
                  value["point"]["y"].asDouble(),
                  0);
    std::cout<<p.x<<" "<<p.y<<std::endl;

    p.x = p.x - 1.5;
    p.x = -((p.x)/0.02 - 600);
    p.y = -((p.y)/0.02 - 600);

//    std::cout<<p.x<<" "<<p.y<<std::endl;

    cv::Point2f r;
    r.x = p.y * (1200/256);
    r.y = p.x * (1200/256);

    std::cout<<r.x<<" "<<r.y<<std::endl;

    return r;

}

int main1()
{
    cv::Mat image = cv::imread("/home/data/workspace/bev/1669975132945161.jpg");

    auto root = load_json("/home/data/workspace/hibag_HP-30-V71-R-008_default_000_20221202175852_20221202175927/fusion_parkspace/1669975132927161.json");
    std::cout<<root["local_parking_slots"]<<std::endl;
    for(const auto& slot: root["parking_slots"]["local_parking_slots"])
    {
        std::vector<cv::Point2f> pts;
        pts.push_back(parse(slot["front_right"]));
        pts.push_back(parse(slot["front_left"]));
        pts.push_back(parse(slot["rear_right"]));
        pts.push_back(parse(slot["rear_left"]));

        cv::line(image, pts[0], pts[1], cv::Scalar(0, 0, 255), 2);
        cv::line(image, pts[1], pts[2], cv::Scalar(0, 0, 255), 2);
        cv::line(image, pts[2], pts[3], cv::Scalar(0, 0, 255), 2);
        cv::line(image, pts[3], pts[0], cv::Scalar(0, 0, 255), 2);
    }

    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::imshow("test", image);
    cv::waitKey(0);

    return 0;
}

class Timer
{
public:
    typedef std::chrono::steady_clock::time_point TimePoint;

    /**
     * @brief Construct a new Timer object
     *
     */
    Timer() : t1(std::chrono::steady_clock::now()), t2(std::chrono::steady_clock::now())
    {
    }

    /**
     * @brief
     *
     */
    void Tic()
    {
        t1 = std::chrono::steady_clock::now();
    }
    /**
     * @brief
     *
     */
    void Toc()
    {
        t2 = std::chrono::steady_clock::now();
    }
    /**
     * @brief
     *
     * @return double
     */
    double Duration()
    {
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        return time_used.count();
    }

private:
    TimePoint t1, t2;
};

int main()
{
    haomo::stitch::CameraParam camera_params[4];
    {
        haomo::stitch::CameraParam front_fisheye_camera_params;
        //front
        front_fisheye_camera_params.fx_ = 470.145721;
        front_fisheye_camera_params.fy_ = 470.931427;
        front_fisheye_camera_params.cx_ = 949.934387;
        front_fisheye_camera_params.cy_ = 542.910217;

        front_fisheye_camera_params.k2_ = 0.0637737662;
        front_fisheye_camera_params.k3_ = -0.0107594291;
        front_fisheye_camera_params.k4_ = 0.00140405376;
        front_fisheye_camera_params.k5_ = -0.000608808594;

        front_fisheye_camera_params.image_width_ = 1920;
        front_fisheye_camera_params.image_height_ = 1080;

        front_fisheye_camera_params.camera_qvec_[0] = -0.36211431;
        front_fisheye_camera_params.camera_qvec_[1] = 0.618382394;
        front_fisheye_camera_params.camera_qvec_[2] = -0.605988503;
        front_fisheye_camera_params.camera_qvec_[3] = 0.345332325;


        front_fisheye_camera_params.camera_tvec_[0] = 3.90823197;
        front_fisheye_camera_params.camera_tvec_[1] = 0.0178015232;
        front_fisheye_camera_params.camera_tvec_[2] = 0.501123488;

        camera_params[0] = front_fisheye_camera_params;
    }

    {
        haomo::stitch::CameraParam left_fisheye_camera_params;
        //left
        left_fisheye_camera_params.fx_ = 470.09082;
        left_fisheye_camera_params.fy_ = 470.961731;
        left_fisheye_camera_params.cx_ = 963.229614;
        left_fisheye_camera_params.cy_ = 537.852905;

        left_fisheye_camera_params.k2_ = 0.0646895617;
        left_fisheye_camera_params.k3_ = -0.0112142526;
        left_fisheye_camera_params.k4_ = 0.00236971141;
        left_fisheye_camera_params.k5_ = -0.000936869124;

        left_fisheye_camera_params.image_width_ = 1920;
        left_fisheye_camera_params.image_height_ = 1080;

        left_fisheye_camera_params.camera_qvec_[0] = 0.551497;
        left_fisheye_camera_params.camera_qvec_[1] = -0.83377164;
        left_fisheye_camera_params.camera_qvec_[2] = -0.0243668;
        left_fisheye_camera_params.camera_qvec_[3] = -0.00906921457;

        left_fisheye_camera_params.camera_tvec_[0] = 2.12899709;
        left_fisheye_camera_params.camera_tvec_[1] = 0.990905285;
        left_fisheye_camera_params.camera_tvec_[2] = 0.663555205;

        camera_params[3] = left_fisheye_camera_params;
    }
//
    {
        haomo::stitch::CameraParam rear_fisheye_camera_params;
        //rear
        rear_fisheye_camera_params.fx_ = 470.77243;
        rear_fisheye_camera_params.fy_ = 471.311188;
        rear_fisheye_camera_params.cx_ = 960.65625;
        rear_fisheye_camera_params.cy_ = 539.924927;

        rear_fisheye_camera_params.k2_ = 0.0634473041;
        rear_fisheye_camera_params.k3_ = -0.00634438358;
        rear_fisheye_camera_params.k4_ = -0.0021761669;
        rear_fisheye_camera_params.k5_ = 0.000430867862;

        rear_fisheye_camera_params.image_width_ = 1920;
        rear_fisheye_camera_params.image_height_ = 1080;

        rear_fisheye_camera_params.camera_qvec_[0] = -0.382835537;
        rear_fisheye_camera_params.camera_qvec_[1] = 0.581698775;
        rear_fisheye_camera_params.camera_qvec_[2] = 0.599986613;
        rear_fisheye_camera_params.camera_qvec_[3] = -0.393801481;

        rear_fisheye_camera_params.camera_tvec_[0] = -0.941009462;
        rear_fisheye_camera_params.camera_tvec_[1] = -0.100865208;
        rear_fisheye_camera_params.camera_tvec_[2] = 0.656048536;

        camera_params[2] = rear_fisheye_camera_params;
    }
//
    {
        haomo::stitch::CameraParam right_fisheye_camera_params;;
        //right
        right_fisheye_camera_params.fx_ = 469.435394;
        right_fisheye_camera_params.fy_ = 469.852112;
        right_fisheye_camera_params.cx_ = 967.581665;
        right_fisheye_camera_params.cy_ = 538.946777;

        right_fisheye_camera_params.k2_ = 0.0640982;
        right_fisheye_camera_params.k3_ = -0.0122619588;
        right_fisheye_camera_params.k4_ = 0.00125413574;
        right_fisheye_camera_params.k5_ = -0.000395158;

        right_fisheye_camera_params.image_width_ = 1920;
        right_fisheye_camera_params.image_height_ = 1080;

        right_fisheye_camera_params.camera_qvec_[0] = -0.0140862009;
        right_fisheye_camera_params.camera_qvec_[1] = -0.0182368234;
        right_fisheye_camera_params.camera_qvec_[2] = 0.821001172;
        right_fisheye_camera_params.camera_qvec_[3] = -0.570461273;

        right_fisheye_camera_params.camera_tvec_[0] = 2.14122534;
        right_fisheye_camera_params.camera_tvec_[1] = -0.929897547;
        right_fisheye_camera_params.camera_tvec_[2] = 0.640055537;

        camera_params[1] = right_fisheye_camera_params;
    }

    void * void_camera_params[4];
    for (int cid = 0; cid < 4; cid++) {
        void_camera_params[cid] = (void *)&(camera_params[cid]);
    }

    haomo::stitch::CarParam car_param;

    car_param.car_length_ = 5.0;
    car_param.car_width_ = 1.891;
    car_param.rearshaft2carcenter_ = 1.390;
    car_param.wheel_diameter = 0.786;


    haomo::stitch::StitchParam stitch_param;
    // if (!readStitchParam(default_stitch_yaml, stitch_param)) {
    {
        stitch_param.use_car_rect_  = true;
        stitch_param.image_width_  = 750;
        stitch_param.image_height_  = 750;
        stitch_param.pix_resolution_ = 0.02;
        stitch_param.extend_pix_ = 0;
    }

    stitch_param.lean_angle_ = 5.0;//lean_angle;
    stitch_param.use_car_rect_ = true;
    int     width = stitch_param.image_width_;
    int     height = stitch_param.image_height_;
    cv::Mat stitch_mat = cv::Mat::zeros(height, width, CV_8UC2);
    stitch_mat.setTo(cv::Vec2b(128, 0));

    haomo::stitch::DataType process_data_type = haomo::stitch::DataType::DT_YUYV;
    uint8_t *stitch_result = stitch_mat.data;

    haomo::stitch::LutInfo lut_info;
    haomo::stitch::IPM_generate_lut(void_camera_params, car_param, &stitch_param, &lut_info, process_data_type);

    int                  camera_nbr = 4;
    std::vector<std::string>       file_names = {"front", "right", "back", "left"};
    std::vector<cv::Mat> inputs(camera_nbr);
    uint8_t *            stitch_data[camera_nbr];

//    uchar y1, u1, v1, y2, u2, v2;
//    RGB2YUV(0, 0, 0, &y1, &u1, &v1);
//
//    std::cout<<(int)y1<<" "<<(int)u1<<" "<<(int)v1<<std::endl;

    inputs[0] = cv::imread("/home/data/CLionProjects/stitch/resource/front_fisheye_camera.jpg");
//    stitch_data[0] = inputs[0].data;

    inputs[1] = cv::imread("/home/data/CLionProjects/stitch/resource/right_fisheye_camera.jpg");
//    stitch_data[1] = inputs[1].data;

    inputs[2] = cv::imread("/home/data/CLionProjects/stitch/resource/rear_fisheye_camera.jpg");
//    stitch_data[2] = inputs[2].data;

    inputs[3] = cv::imread("/home/data/CLionProjects/stitch/resource/left_fisheye_camera.jpg");

    std::vector<cv::Mat> inputs_uyvy(camera_nbr);
    for(int i = 0; i < 4; ++i)
    {
        convertToUYVY(inputs[i], inputs_uyvy[i]);
        stitch_data[i] = inputs_uyvy[i].data;
    }
    for(int i = 0; i < 1000; ++i)
    {
        Timer timer;
        timer.Tic();
        haomo::stitch::IPM_stitch(stitch_data, &lut_info, stitch_result);
        timer.Toc();
        std::cout<<"IPM cost: "<<timer.Duration()<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    haomo::stitch::IPM_stitch(stitch_data, &lut_info, stitch_result);

    cv::Mat dst = cv::Mat(height, width, CV_8UC2, stitch_result);
    cv::Mat dst_bgr;
    for(int i = 0; i < 1000; ++i)
    {
        Timer timer;
        timer.Tic();
        cv::cvtColor(dst, dst_bgr, cv::COLOR_YUV2RGB_UYVY);
        timer.Toc();
        std::cout<<"cvt color cost: "<<timer.Duration()<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cv::imshow("test", dst_bgr);
    cv::waitKey(0);


    return 0;
}
