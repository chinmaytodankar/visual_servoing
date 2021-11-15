using namespace std;
using namespace ros;
using namespace cv;

static const string IMAGE_WITH_CENTERS_WINDOW = "Cirlce Centers Image";

// HSV Threshold Values for All Circles
static const Scalar redLow = Scalar(0,200,50);
static const Scalar redHigh = Scalar(5,255,255);
static const Scalar blueLow = Scalar(110,200,50);
static const Scalar blueHigh = Scalar(130,255,255);
static const Scalar greenLow = Scalar(50,200,50);
static const Scalar greenHigh = Scalar(70,255,255);
static const Scalar purpleLow = Scalar(140,200,50);
static const Scalar purpleHigh = Scalar(160,255,255);

// Color Values
static const Scalar blackColor = Scalar(0, 0, 0);       // BGR

Point redCenter;
Point blueCenter;
Point greenCenter;
Point purpleCenter;

Point referenceCenters [4];
Point currentCenters [4];

double** errors = new double*[8];

Mat image, imageWithCenters;
string s = "src/vision_based_manipulation/reference.jpg";

bool spinOnceFlag = false;
bool visualServoingFlag = false;

double** lLe = new double*[2];

double** inverseJacobian = new double*[2];

ServiceClient switchControllerService;
Subscriber jointStateSubscriber;

double q1Angle, q2Angle;

double link1Length = 0.5, link2Length = 0.5, link3Length = 0.5;
double lambda = 0.00125;
double f = 1;
double z = 1;

Publisher pub_q1_pos;
Publisher pub_q2_pos;

Publisher pub_q1_vel;
Publisher pub_q2_vel;