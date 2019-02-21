
#include "GMR.h"

#include "MathLib/MathLib.h"


using namespace MathLib;





std::vector< std::vector<double> > handConfiguations(int hand, int nb_classes){
    /*
     * This function returns the final configurations of the joints of the Allegro hand.
     * The number of rows of the returned matrix is nb_classes+1. The extra row is for
     * the idle state (open palm) and it is always stored in the first row of the matrix.
     * The second row corresponds to class 1, the third row corresponds to class 2, etc.
     * The classes are considered in the order: precision disk, tripod, thumb-2 fingers,
     * thumb-4 fingers, ulnar grasp.
     *
     * The input hand corresponds to the choice of hand:
     *
     *      0: left hand
     *      1: right hand
     *  other: it returns the configuration of the idle state (open palm)
     *
     *
     */

    int nbAllegroJoints=16;     // number of the joints of the Allegro hand

    std::vector< std::vector<double> > configuration(nb_classes+1,std::vector<double>(nbAllegroJoints));



    switch (hand+1) {
    case 1:
        {
        // set the configurations for the left hand

        double pos0[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8294999999999999, 0.0, 0.0, 0.0};
        configuration[0].assign(pos0,pos0+nbAllegroJoints);

        double pos1[]={-0.04237095869508623, 0.5470453333881532, 0.7437613579286353, 0.8246622519939837, 0.028702179278624937, 0.21415387491454108, 1.0614804723606348, 0.37650835547609685, 0.1284833816946608, 0.41054355747717247, 1.1237207395184723, 0.42723059373656896, 1.3493727871788868, 0.28406342362896075, -0.08937179089510226, 1.257289923016124};

        configuration[1].assign(pos1,pos1+nbAllegroJoints);

        double pos2[]={-0.1271049481804714, 0.7094641788588949, 0.7146115325008343, 0.8238409621831889, -0.07129752052943118, 0.8265867914431462, 0.6850142232442048, 0.7839548430859884, -0.01553320216526276, -0.14385972598926425, 0.022219205813237504, -0.025653460750270383, 1.34823913866758, 0.35977134415753875, 0.12314967349942667, 1.0725716422955927};
        configuration[2].assign(pos2,pos2+nbAllegroJoints);

        double pos3[]={-0.05849754987579765, 1.0670753939401456, 0.6826479885104186, 0.8606031944835235, 0.00481340303919539, 1.0466559414495369, 0.8283367339626594, 0.8768701371658273, -0.015603029451737777, -0.14356221306852357, 0.021642462422132115, -0.02650620338071224, 1.3198479914680814, 0.2713195831034956, -0.013571358366547017, 1.2570757463955298};
        configuration[3].assign(pos3,pos3+nbAllegroJoints);

        double pos4[]={-0.08313273406077283, 1.0626900627500735, 0.6776767577766083, 0.8689894857574274, 0.01085244168119158, 1.0478904498503627, 0.8153515553104972, 0.8706244017438871, 0.08474530007832862, 0.9757269131410359, 0.8916851570471536, 0.6973578145107358, 1.3767011967950054, 0.31613809911144924, 0.15428263805282766, 0.9440189891989741};
        configuration[4].assign(pos4,pos4+nbAllegroJoints);

        double pos5[]={-0.05012838772059667, -0.13433793254175375, -0.04671840751037163, 0.04517307903339739, 0.009254868475132948, -0.2005495252070131, 0.30151401910510633, 0.028854511477014885, -0.32906246400334416, 0.9333724582174228, 1.1422871771730567, 0.212455065212159, 1.3716523471127307, 0.46959754664105635, 0.17048521323225668, 1.4742483963649067};
        configuration[5].assign(pos5,pos5+nbAllegroJoints);
        }
        break;
    case 2:
        {
        // set the configurations for the right hand

        double pos0[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.821205, 0.0, 0.0, 0.0};
        configuration[0].assign(pos0,pos0+nbAllegroJoints);

        double pos1[]={0.0, 0.6366369240000003, 0.7728741899999998, 0.7656254100000002, 0.0, 0.5352607260000002, 0.8123945939999999, 0.80434827, 0.0, 0.5865746039999999, 0.8123945939999999, 0.9586917449999999, 1.342557216, 0.03739903200000002, 0.068395536, 0.7969718789999998};
        configuration[1].assign(pos1,pos1+nbAllegroJoints);

        double pos2[]={0.0, 0.788075442, 0.7987861530000002, 0.8428884750000004, 0.0, 0.750349908, 0.7728741900000002, 0.80434827, 0.0851499, 0.019976418000000017, -0.08035641899999998, 0.058202595000000044, 1.366224453, 0.11711185200000002, 0.272908845, 0.7969718789999998};
        configuration[2].assign(pos2,pos2+nbAllegroJoints);

        double pos3[]={0.0, 1.1154472560000002, 0.6548722290000002, 0.8428884750000004, 0.0, 1.25329743, 0.35287668900000013, 0.8686428300000004, 0.0851499, 0.019976418000000017, -0.08035641899999998, 0.058202595000000044, 1.366224453, 0.16129911600000005, 0.2600246880000001, 0.7969718789999998};
        configuration[3].assign(pos3,pos3+nbAllegroJoints);

        double pos4[]={0.0, 1.1154472560000002, 0.6548722290000002, 0.8428884750000004, 5e-324, 1.2790437659999998, 0.35287668900000013, 0.9844461000000003, 5e-324, 1.1783827440000003, 0.549546624, 0.7142993549999999, 1.366224453, 0.16129911600000005, 0.2600246880000001, 0.7969718789999998};
        configuration[4].assign(pos4,pos4+nbAllegroJoints);

        double pos5[]={0.10584400000000005, 0.08374940000000003, -0.0003873999999999822, 0.0, 0.0, 0.0, -0.0003873999999999822, 0.0, 0.364062, 1.0376786000000002, 0.7675000000000001, 0.4226245000000002, 1.396, 0.4753636, 0.4822446000000001, 0.8050221};
        configuration[5].assign(pos5,pos5+nbAllegroJoints);
        }
        break;
    default:
        double pos0[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.821205, 0.0, 0.0, 0.0};
        configuration[0].assign(pos0,pos0+nbAllegroJoints);
        configuration[1].assign(pos0,pos0+nbAllegroJoints);
        configuration[2].assign(pos0,pos0+nbAllegroJoints);
        configuration[3].assign(pos0,pos0+nbAllegroJoints);
        configuration[4].assign(pos0,pos0+nbAllegroJoints);
        configuration[5].assign(pos0,pos0+nbAllegroJoints);

        break;
    }


    return configuration;

}

