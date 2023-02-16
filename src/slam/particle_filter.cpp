#include <cmath>
#include <random>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <chrono>
#include <algorithm>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;

    posteriorPose_ = pose;
    for(auto &&p : posterior_) {
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map,
                                        const pose_xyt_t* true_pose 
                                        )
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anythinupdatefilterg.

std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);

        //float dx = (odometry.x - posteriorPose_.x) * (odometry.x - posteriorPose_.x);
        //float dy = (odometry.y - posteriorPose_.y) * (odometry.y - posteriorPose_.y);
        //float dtheta = fabs(odometry.theta - posteriorPose_.theta);
        //while(dtheta > M_PI) dtheta -= M_PI;
        //std::cout << odometry.utime << "," << sqrt(dx + dy) << std::endl;
        //std::cout << odometry.utime << "," << dtheta << std::endl;

        float dx = (true_pose->x - posteriorPose_.x) * (true_pose->x - posteriorPose_.x);
        float dy = (true_pose->y - posteriorPose_.y) * (true_pose->y - posteriorPose_.y);
        float dtheta = fabs(true_pose->theta - posteriorPose_.theta);
        while(dtheta > M_PI) dtheta -= M_PI;
        //std::cout << true_pose->utime << "," << sqrt(dx + dy) << std::endl;
        std::cout << true_pose->utime << "," << dtheta << std::endl;

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }
    
    posteriorPose_.utime = odometry.utime;

    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    /*
    std::vector<particle_t> prior = posterior_;
    double sampleWeight = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    // TODO: Replace with low variance sample
    std::normal_distribution<> dist(0.0, 0.04);

    for(auto& p: prior) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
    }
    */
    std::vector<particle_t> prior = posterior_;
    double sampleWeight = 1.0/kNumParticles_;
    float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/sampleWeight));
    float c = posterior_[0].weight;
    int i = 0;

    for(int m = 0; m < prior.size(); m++) {
        float U = r + m * sampleWeight;
        while(U > c) {
            i+=1;
            c += posterior_[i].weight;
        }

        auto& p = prior[m];
        p.pose.x = posterior_[i].pose.x;
        p.pose.y = posterior_[i].pose.y;
        p.pose.theta = posterior_[i].pose.theta;
        p.pose.utime = posterior_[i].pose.utime;
        p.parent_pose = posterior_[i].parent_pose;
        p.weight = sampleWeight;
    }
    /*
    std::vector<particle_t> prior = posterior_;
    std::sort(prior.begin(), prior.end(), [] (particle_t &a, particle_t &b) { return a.weight > b. weight; } );
    */
    //for(int i = 0; i <)

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for( auto& p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double sumWeights = 0.0;
    for(auto &&p : proposal) {
        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto &&p : posterior) {
        p.weight /= sumWeights;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;

    /*
    // mean
    for(auto &p : posterior) {
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
    }    

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);
*/
    // MAP
    /*
    particle_t map_particle = posterior[0]; 
    for(auto &p : posterior) {
        if(p.weight > map_particle.weight) 
            map_particle = p;
    }
    pose = map_particle.pose;
    */
    // top k mean
    std::vector<particle_t> sorted_particles = posterior_;
    std::sort(sorted_particles.begin(), sorted_particles.end(), 
        [] (particle_t &a, particle_t &b) { return a.weight > b. weight; } );
    float sumWeight = 0.0;
    for(size_t i = 0; i < static_cast<size_t>(kNumParticles_ * 0.1); i++) {
        auto &p = sorted_particles[i];
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
        sumWeight += p.weight;
    }
    pose.x = xMean / sumWeight;
    pose.y = yMean / sumWeight;
    pose.theta = std::atan2(sinThetaMean / sumWeight, cosThetaMean / sumWeight);

    return pose;
}
