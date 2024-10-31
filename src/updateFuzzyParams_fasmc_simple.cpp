#include <ros/ros.h>
#include <smm_control/FasmcError.h>
#include <smm_control/FasmcFuzzyParams.h>  // Custom message with alpha, beta, gamma
#include <fl/Headers.h>  // Fuzzylite library
#include <vector>
#include "smm_control/timing.h"

// Fuzzy logic variables
fl::Engine* engine;
fl::InputVariable* positionError[3];  // Input for each joint position error
fl::InputVariable* velocityError[3];  // Input for each joint velocity error
fl::OutputVariable* alpha[3];         // Alpha for each joint
fl::OutputVariable* beta[3];          // Beta for each joint
fl::OutputVariable* gammaParams[3];   // Gamma for each joint

ros::Publisher fuzzy_param_pub;

// Function to set up the Fuzzylite engine and define the rules
void setupFuzzyEngine() {
    engine = new fl::Engine;
    engine->setName("FuzzyParamController");

    for (int i = 0; i < 3; ++i) {
        // Define input variables for each joint’s position and velocity error
        positionError[i] = new fl::InputVariable;
        positionError[i]->setName("PositionError" + std::to_string(i + 1));
        positionError[i]->setRange(-1.0, 1.0);

        // Using Gaussian membership functions as specified
        positionError[i]->addTerm(new fl::Gaussian("PE1",  0.00,  0.01));    // ZERO  [mean,stdev]
        positionError[i]->addTerm(new fl::Gaussian("PE2", -0.05, 0.04));   // SMALL_NEGATIVE
        positionError[i]->addTerm(new fl::Gaussian("PE3",  0.05, 0.04));   // SMALL_POSITIVE
        positionError[i]->addTerm(new fl::Gaussian("PE4", -0.25, 0.15));  // MEDIUM_NEGATIVE 
        positionError[i]->addTerm(new fl::Gaussian("PE5",  0.25, 0.15));   // MEDIUM_POSITIVE
        positionError[i]->addTerm(new fl::Gaussian("PE6", -1.00, 0.75));  // LARGE_NEGATIVE
        positionError[i]->addTerm(new fl::Gaussian("PE7",  1.00, 0.75));   // LARGE_POSITIVE    
        engine->addInputVariable(positionError[i]);

        velocityError[i] = new fl::InputVariable;
        velocityError[i]->setName("VelocityError" + std::to_string(i + 1));
        velocityError[i]->setRange(-1.0, 1.0);
        velocityError[i]->addTerm(new fl::Gaussian("VE1", 0.0, 0.1));      // ZERO 
        velocityError[i]->addTerm(new fl::Gaussian("VE2", -2.0, 1.5));     // SMALL_NEGATIVE  
        velocityError[i]->addTerm(new fl::Gaussian("VE3", 2.0, 1.5));      // SMALL_POSITIVE 
        velocityError[i]->addTerm(new fl::Gaussian("VE4", -10.0, 8.0));    // LARGE_NEGATIVE
        velocityError[i]->addTerm(new fl::Gaussian("VE5", 10.0, 8.0));     // LARGE_POSITIVE  
        engine->addInputVariable(velocityError[i]);

        // Define output variables for each joint’s alpha, beta, gamma with specified ranges
        alpha[i] = new fl::OutputVariable;
        alpha[i]->setName("Alpha" + std::to_string(i + 1));
        alpha[i]->setRange(0.01, 1.0);
        alpha[i]->setDefuzzifier(new fl::Centroid(100));  
        alpha[i]->setDefaultValue(fl::nan);               
        alpha[i]->setLockPreviousValue(false);           
        alpha[i]->setAggregation(new fl::Maximum());
        alpha[i]->addTerm(new fl::Gaussian("AO1", 0.001, 0.0005));  // 0.001, 0.0005
        alpha[i]->addTerm(new fl::Gaussian("AO2", 0.010, 0.005));    // 0.01, 0.005
        alpha[i]->addTerm(new fl::Gaussian("AO3", 0.10, 0.05));      // 0.1, 0.05
        engine->addOutputVariable(alpha[i]);

        beta[i] = new fl::OutputVariable;
        beta[i]->setName("Beta" + std::to_string(i + 1));
        beta[i]->setRange(0.01, 1.0);
        beta[i]->addTerm(new fl::Gaussian("BO1", 0.001, 0.005)); // 0.001, 0.0005
        beta[i]->addTerm(new fl::Gaussian("BO2", 0.010, 0.005));   // 0.01, 0.005
        beta[i]->addTerm(new fl::Gaussian("BO3", 0.10, 0.05));     // 0.1, 0.05
        beta[i]->setDefuzzifier(new fl::Centroid(100));  
        beta[i]->setDefaultValue(fl::nan);               
        beta[i]->setLockPreviousValue(false);
        beta[i]->setAggregation(new fl::Maximum()); 
        engine->addOutputVariable(beta[i]);

        gammaParams[i] = new fl::OutputVariable; 
        gammaParams[i]->setName("Gamma" + std::to_string(i + 1));
        gammaParams[i]->setRange(0.01, 1.0);
        gammaParams[i]->addTerm(new fl::Gaussian("GO1", 0.1, 0.05));  // 0.1, 0.05
        gammaParams[i]->addTerm(new fl::Gaussian("GO2", 0.25, 0.15)); // 0.25, 0.15
        gammaParams[i]->addTerm(new fl::Gaussian("GO3", 0.50, 0.25)); // 0.50, 0.25
        gammaParams[i]->setDefuzzifier(new fl::Centroid(100));  
        gammaParams[i]->setDefaultValue(fl::nan);               
        gammaParams[i]->setLockPreviousValue(false);
        gammaParams[i]->setAggregation(new fl::Maximum());
        engine->addOutputVariable(gammaParams[i]);

        // Define fuzzy rules for each joint
        fl::RuleBlock* ruleBlock = new fl::RuleBlock;
        ruleBlock->setName("ruleBlock");
        ruleBlock->setDescription("");
        ruleBlock->setEnabled(true);
        ruleBlock->setConjunction(new fl::Minimum());  
        ruleBlock->setDisjunction(new fl::Maximum());  
        ruleBlock->setImplication(new fl::Minimum()); 
        ruleBlock->setActivation(new fl::General());   

        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE1 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO1 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO1", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE1 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO1 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO1", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE1 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO1 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO1", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE1 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE1 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE2 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO1", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE2 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE2 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE2 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE2 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE3 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO1", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE3 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE3 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE3 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE3 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE4 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE4 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE4 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE4 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE4 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE5 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE5 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE5 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE5 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE5 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE6 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE6 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE6 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE6 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE6 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE7 and VelocityError" + std::to_string(i + 1) + " is VE1 then Alpha" + std::to_string(i + 1) + " is AO2 and Beta" + std::to_string(i + 1) + " is BO1 and Gamma" + std::to_string(i + 1) + " is GO2", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE7 and VelocityError" + std::to_string(i + 1) + " is VE2 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE7 and VelocityError" + std::to_string(i + 1) + " is VE3 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO2 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE7 and VelocityError" + std::to_string(i + 1) + " is VE4 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));
        ruleBlock->addRule(fl::Rule::parse("if PositionError" + std::to_string(i + 1) + " is PE7 and VelocityError" + std::to_string(i + 1) + " is VE5 then Alpha" + std::to_string(i + 1) + " is AO3 and Beta" + std::to_string(i + 1) + " is BO3 and Gamma" + std::to_string(i + 1) + " is GO3", engine));

        engine->addRuleBlock(ruleBlock);
    }
}

// Callback function for /fasmc_error_state
void errorStateCallback(const smm_control::FasmcError::ConstPtr& msg) {
    if (msg->position_error.size() != 3 || msg->velocity_error.size() != 3) {
        ROS_ERROR("Expected 3 values for position and velocity errors.");
        return;
    }

    smm_control::FasmcFuzzyParams fuzzy_params_msg;

    for (int i = 0; i < 3; ++i) {
        // Set fuzzy inputs for the current joint
        positionError[i]->setValue(msg->position_error[i]);  // updated to setValue()
        velocityError[i]->setValue(msg->velocity_error[i]);  // updated to setValue()

        // Process fuzzy logic for each joint
        engine->process();

        // Get the fuzzy output values for alpha, beta, gamma
        fuzzy_params_msg.alpha[i] = alpha[i]->getValue();         // updated
        fuzzy_params_msg.beta[i] = beta[i]->getValue();           // updated
        fuzzy_params_msg.gamma[i] = gammaParams[i]->getValue();   // updated
    }

    // Publish the fuzzy parameters
    fuzzy_param_pub.publish(fuzzy_params_msg);

    ROS_INFO_STREAM("Published Fuzzy Params - Alpha: [" << fuzzy_params_msg.alpha[0] << ", " << fuzzy_params_msg.alpha[1] << ", " << fuzzy_params_msg.alpha[2] 
                     << "], Beta: [" << fuzzy_params_msg.beta[0] << ", " << fuzzy_params_msg.beta[1] << ", " << fuzzy_params_msg.beta[2] 
                     << "], Gamma: [" << fuzzy_params_msg.gamma[0] << ", " << fuzzy_params_msg.gamma[1] << ", " << fuzzy_params_msg.gamma[2] << "]");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateFuzzyParams_fasmc_simple");
    ros::NodeHandle nh;

    setupFuzzyEngine();

    ros::Subscriber error_state_sub = nh.subscribe("/fasmc_error_state", 10, errorStateCallback);
    fuzzy_param_pub = nh.advertise<smm_control::FasmcFuzzyParams>("/fasmc_fuzzy_params", 10);

    ros::Rate loop_rate(UPDATE_FUZZY_PARAMS_RATE);

    while (ros::ok()) {
        //setupFuzzyEngine();  // Publish the current desired state
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
