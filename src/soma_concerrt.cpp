#include "soma_concerrt.h"

SomaConcerrtResult SomaConcerrt::solve(int task)
{

  bool solved = rl::plan::Concerrt::solve();

  rl::plan::Policy policy;

  getPolicy(policy);
  policy.exportGraphToFile("");
  std::cout << "Solution " << solved << std::endl << std::flush;

  return SomaConcerrtResult(policy, solved);
}

void SomaConcerrt::choose(rl::math::Vector& chosen)
{

  ::boost::uniform_real< ::rl::math::Real > goalDistr(0.0f, 1.0f);  
  if (goalDistr(*this->gen) > 0.2)
  {

  //for (unsigned i = 0; i < maximum_choose_attempts; ++i)
    while (true)
    {
      auto sampled_pose = ROI_sampler->generate(sample_01);

      auto result = jacobian_controller->moveSingleParticle(ROI_sampler_reference,
                                                            sampled_pose,
                                                            collisions_ignored);
      if (result)
      {
        chosen = result.trajectory.back();
        return;
      }
    }
  }
  else
  {

    auto sampled_pose = goalBiased_sampler->generate(sample_01);

    auto result = jacobian_controller->moveSingleParticle(ROI_sampler_reference,
                                                          sampled_pose,
                                                          collisions_ignored);

    chosen = result.trajectory.back();
    return;
  }


    throw std::runtime_error("Choose unable to sample");
}

void SomaConcerrt::sampleInitialParticles(std::vector<rl::plan::Particle>& initial_particles, const rl::math::Vector&)
{
  initial_particles.resize(nrParticles);
  for (unsigned i = 0; i < nrParticles; ++i)
  {
    auto& particle = initial_particles[i];

    particle.config = start_configurations[i];
    particle.ID = i;
    model->setPosition(particle.config);
    model->updateFrames();
    model->isColliding();
    particle.contacts = model->scene->getLastCollisions();
  }
}

std::string printContacts(std::unordered_set<std::pair<std::string, std::string>> cList)
{
  std::string s;
  for (auto& c : cList)
    s += " " + c.first;
  return s;
}

bool SomaConcerrt::isAdmissableGoal(boost::shared_ptr<rl::plan::BeliefState> belief)
{
  //TODO why was this different then CERRT
  auto& particles = belief->getParticles();
  // for every particle, check that all required contact pairs are present, and the pose lies within a workspace
  // goal manifold
  for (auto& particle : particles)
  {
    auto nonpresent_contacts = required_goal_contacts;
    for (auto& contact_and_description : particle.contacts)
      nonpresent_contacts.erase(contact_and_description.first);

    if (!nonpresent_contacts.empty())
    {
      std::cout << "missing contact:"<< printContacts(nonpresent_contacts)  << std::endl << std::flush;
      return false;
    }

    model->setPosition(particle.config);
    model->updateFrames();
    if (!goal_manifold_checker->contains(model->forwardPosition()))
    {
      std::cout << "not on manifold" << std::endl << std::flush;
      return false;
    }
  }
  return true;
}


bool SomaConcerrt::goalConnect(Vertex newVertex,
                               bool addToGraph,
                               ::rl::math::Vector3& slidingNormal,
                               ::rl::math::Transform& goalT,
                               const int graphID)
{
      if (isAdmissableGoal((*this->graphs[graphID])[newVertex].beliefState))
      {

          (*this->graphs[graphID])[newVertex].onSolutionPath = true;
          (*this->graphs[graphID])[newVertex].atGoal = true;
          return true;
      }
      // if not at goal try to get there with a Jacobian controller and sample if needed K-times
      else
      {
        /*
        ::std::vector<rl::plan::Particle> particles_new;
        ::std::vector<rl::plan::Particle> particles = (*this->graphs[graphID])[newVertex].beliefState->getParticles();
        for (auto p : particles)
        {
          this->model->setPosition(p.config);
          this->model->updateFrames();
          ::rl::math::Transform pTransform = this->model->forwardPosition();
          Eigen::Vector3d vz(0,0,-0.5);
          ::rl::math::Transform gTransform = pTransform * Eigen::Translation<double,3>(0, 0, -1);


          auto result = jacobian_controller->moveSingleParticle(p.config,
                                                                gTransform,
                                                                collisions_ignored);
          if (!result)
          {
            std::cout << "particle goal connect failed\n"<< std::flush;
            return false;
          }
          else
          {
            //utilities::eigenToStd(result.trajectory.back());
            model->setPosition(result.trajectory.back());
            model->updateFrames();
            model->isColliding();
            initColls = this->scene->getLastCollisions();
            p_init.contacts = initColls;
            rl::plan::Particle p_new(result.trajectory.back(), );
          }*/

        }
//        if(addToGraph)
//        {
//          return true;
//        }
//      }
      return false;
}

rl::plan::NoisyModel::ActionType
SomaConcerrt::selectAction(const Graph& graph, const Vertex& nearest)
{
  if( graph[nearest].beliefState->isInCollision())
  {
    // randomly decide to do a slide or not
    boost::random::uniform_01<boost::random::mt19937> doSlideDistr(*this->gen);
    bool doSlide = doSlideDistr() < this->gamma;
    if (doSlide)
    {
      // TODO this is not in CERRT right now. We can add  to the planner if needed
      //      if(doSlideDistr() < this->gamma)
      //        return GUARDEDSLIDE;
      //      else
      return rl::plan::NoisyModel::CONNECTSLIDE;
    }
    else
    {
      return rl::plan::NoisyModel::CONNECT;
    }
  }
  else
  {
    // randomly decide to do a guarded move or not
    boost::random::uniform_01<boost::random::mt19937> doGuardDistr(*this->gen);
    bool doGuardedMove = doGuardDistr() < this->gamma;

    if (doGuardedMove)
    {
      return rl::plan::NoisyModel::GUARDED;
    }
    else
    {
      return rl::plan::NoisyModel::CONNECT;
    }
  }
}

// one step reactive exploration in a given graph
bool SomaConcerrt::expandTreeReactively(const int graphID,
                                    const Vertex& leafVertex,
                                    std::vector<Vertex>& leafVertexes_inLocalTree,
                                    std::vector<goalConnectType>& goalConnections)
{
  bool solutionFound =  false;
  int mainGraph = 0;
  ::rl::math::Vector3 slidingNormal; // =
  slidingNormal.setZero();

  // the main goal to which we wan to connect
  this->model->setPosition(*this->goal);
  this->model->updateFrames();
  ::rl::math::Transform goalTransform = this->model->forwardPosition();

  // if this is a new tree then init the first node
  if (boost::num_vertices((*this->graphs[graphID])) == 0)
  {
    //return the root of the new graph
    Vertex newsV = initNewGraph(leafVertex, mainGraph, graphID);

    //save the root of each sibling graph
    leafVertexes_inLocalTree.push_back(newsV);

    //all sibs on path - so we show where reconnect fails
    (*this->graphs[mainGraph])[leafVertex].onSolutionPath = true;
    (*this->graphs[graphID])[newsV].onSolutionPath = true;

    //check if we are at goal
    if ((*this->graphs[mainGraph])[leafVertex].atGoal)
      return true;

    // try to connect first node to any vertex on solution path or connect to goal
    // check if we are at goal or connectable to it
    if (goalConnect(newsV, true, slidingNormal, goalTransform, graphID))
    {
      goalConnectType gct;
      gct.connected_to_existing_vertex = false;
      gct.policy_end_vertex = newsV;
      // we are connected to the goal!
      // if there are no out edges, than we are already at the goal
      OutEdgeIteratorPair outEdges = ::boost::out_edges(newsV, (*this->graphs[graphID]));
      if (outEdges.first != outEdges.second)
      {
        Edge e  = *outEdges.first;
        Vertex v = ::boost::target(e, (*this->graphs[graphID]));
        gct.policy_end_vertex = v;
      }

      // save this goal connect solution
      goalConnections[graphID]= gct;
      return true;
    }

    //check if we can connect to any belief on solution path
    for (int i  = 0; i < vertexesOnPath.size(); i++)
    {
      Vertex vG0 = vertexesOnPath[i];
      this->model->setPosition(*(*this->graphs[mainGraph])[vG0].q);
      this->model->updateFrames();
      ::rl::math::Transform beliefGoalTransform = this->model->forwardPosition();

      double maxBeliefDistance = workSpaceDistance((*this->graphs[mainGraph])[newsV].beliefState, beliefGoalTransform );

      if (beliefConnectToBelief(newsV, false, slidingNormal, vG0, graphID, mainGraph))
      {

        goalConnectType gct;
        gct.connected_to_existing_vertex = true;
        gct.existing_vertex = vG0;
        Vertex end_ = copyVertex(vG0, mainGraph, graphID);

        gct.policy_end_vertex = end_;
        Edge e  = this->addEdge(newsV, end_, rl::plan::NoisyModel::CONNECT, (*this->graphs[graphID]), 1.0);
        (*this->graphs[graphID])[e].chosenSampleDelta =
            (*this->graphs[graphID])[end_].beliefState->configMean() -
            (*this->graphs[graphID])[newsV].beliefState->configMean();
        (*this->graphs[graphID])[e].isBeliefConnectAction = true;

        // node is on path
        (*this->graphs[graphID])[newsV].onSolutionPath = true;
        (*this->graphs[graphID])[end_].onSolutionPath = true;
        (*this->graphs[graphID])[end_].parent  = newsV;

        goalConnections[graphID]= gct;
        return true;
      }
    }
  }

  // sample random config
  ::rl::math::Vector chosenSample(this->model->getDof());
  this->choose(chosenSample);
  //Choose node to extend
  Neighbor n = this->nearest((*this->graphs[graphID]), chosenSample);

  //Choose action to execute
  rl::plan::NoisyModel::ActionType u = this->selectAction((*this->graphs[graphID]), n.first);

  ::std::vector<rl::plan::Particle> particles;
  bool splitParticles = false;

  // remove bool flag: splitParticles
  bool sampleResult = this->simulate(u, n, chosenSample,
                                     this->nrParticles,
                                     true, particles,
                                     slidingNormal,
                                     splitParticles,
                                     (*this->graphs[graphID]));

  // init some varaibles
  this->uniqueActionID++;
  bool cluserAtGoal = false;

  if (sampleResult)
  {
    viewer->drawConfiguration(particles[0].config);
    if (!splitParticles)
    {
      // save the way we reacched the goal
      goalConnectType gct;
      // insert vertex and edge for this node, check if we are close to goal and if not, try to connect to goal,
      Vertex newsV;
      if (expandGraph(particles, slidingNormal,
                      n, u, goalTransform,
                      (double)particles.size()/(double)this->nrParticles,
                      graphID, gct, newsV, chosenSample))
      {
        // if goal reached returne true
        goalConnections[graphID]= gct;
        solutionFound = true;
        return true;
      }
    }
    else // if we need to split the particles
    {
      //cluster particles based on #collision, surfacesID, normal, (distance projected on normal)
      ::std::vector<::std::vector<rl::plan::Particle>> particleClusters;
      clusterParticles(particles, particleClusters, true);

      double maxProb = 0, maxUncer = 0, maxHueristic = 0;
      int maxProbI, maxUncerI, maxHueristicI, tmpH;

      bool splitForNN[particleClusters.size()];

      double uncertaitnySumm = 0;
      bool dropCluster = false;
      for (int i = 0; i < particleClusters.size(); i++)
      {
        ::boost::shared_ptr<rl::plan::BeliefState> belief = ::boost::make_shared<rl::plan::BeliefState>( particleClusters[i],
                                                                                     model,
                                                                                     particleClusters[i][0].contacts.begin()->second.normal);

        //this is similar to the uncertaitny measure in the NN funciton
        uncertaitnySumm += sqrt(belief->eeCovariance().trace());

        if (particleClusters[i].size() > maxProb )
        {
          maxProb = particleClusters[i].size();
          maxProbI = i;
        }

        if (sqrt(belief->eeCovariance().trace() > maxUncer))
        {
          maxUncer = sqrt(belief->eeCovariance().trace());
          maxUncerI = i;
        }

        // skip this reactive branching if there is a brench with low probability
        if ((double)particleClusters[i].size() < 0.1*(double)this->nrParticles )
        {
          dropCluster = true;
          splitForNN[i] = false;
        }
        else
          splitForNN[i] = true;

      }

      //add Clusters to graph
      std::vector<Vertex> listOfVertexes;
      for (int i = 0; i < particleClusters.size(); i++)
      {

        double reachability = (double)particleClusters[i].size()/(double)this->nrParticles;
        //repopulate cluster with particles
        resampleMissingParticles(particleClusters[i]);
        //create vertex and edges to parent
        Vertex newVertex = this->addAndGetVertex(particleClusters[i], true, n.first, graphID);

        (*this->graphs[graphID])[newVertex].siblingUncertainty = uncertaitnySumm -
            sqrt((*this->graphs[graphID])[newVertex].beliefState->eeCovariance().trace());

        // the branching factor increases with the number of branching factor of the parent node + the nr of reactive branches - 1
        (*this->graphs[graphID])[newVertex].branchingFactor = (*this->graphs[graphID])[n.first].branchingFactor +particleClusters.size() - 1;

        (*this->graphs[graphID])[newVertex].hasSiblings = true;
        //setup pruned branches - now only prune for max probability
        (*this->graphs[graphID])[newVertex].exludeFromNN = true; //never exlclude = false

        //skip split if there is a branch with less then 10% particles
        if ((i == maxProbI) && splitForNN[i])
          (*this->graphs[graphID])[newVertex].exludeFromNN = false;

        rl::math::Vector3 color;
        color[0] = 0;
        color[1] = 0;
        color[2] = 100;
        //Edge newEdge = this->addEdgeColored(n.first, newVertex, (int)u, this->graphs[graphID], color);
        Edge e = this->addEdge(n.first, newVertex, u, (*this->graphs[graphID]), reachability);
        (*this->graphs[graphID])[e].chosenSampleDelta =
            (*this->graphs[graphID])[newVertex].beliefState->configMean() -
            (*this->graphs[graphID])[n.first].beliefState->configMean();

        //save a list of vertexes so we can connect them in the next step
        listOfVertexes.push_back(newVertex);
        // General workspace distance calculation

        // define goalT
        //double maxError = workSpaceDistance((*this->graphs[graphID])[newVertex].beliefState, goalTransform);

        if ((*this->graphs[graphID])[newVertex].branchingFactor <= this->K_contingency_limit && !solutionFound)
        {
          // workspace goal reachability test
          //if (maxError < this->goalWorkspaceEpsilon)

          // task based goal reachability test
          if (isAdmissableGoal((*this->graphs[graphID])[newVertex].beliefState))
          {
            // visualize goal connect step
            //              if (this->viewer)
            //                this->drawParticles(particles);

            this->end.push_back(newVertex);
            solutionFound = true;
            (*this->graphs[graphID])[newVertex].atGoal = true;
            goalConnectType gct;
            gct.connected_to_existing_vertex = false;
            gct.policy_end_vertex = newVertex;
            goalConnections[graphID]= gct;

            // if this is a contingency branch, don't try to connect any other branch
            // if connected other branches, we cant handle them in the update phase
            solutionFound = true;

          }
          else
          {
            //add K_limit here too
            if (goalConnect(newVertex,
                            true,
                            particleClusters[i][0].contacts.begin()->second.normal,
                            goalTransform,
                            graphID))
            {
              OutEdgeIteratorPair outEdgeIters = ::boost::out_edges(newVertex, (*this->graphs[graphID]));
              for (OutEdgeIterator edgeIt = outEdgeIters.first; edgeIt != outEdgeIters.second; ++edgeIt)
              {
                Vertex v_ = ::boost::target(*edgeIt, (*this->graphs[graphID]));

                if ((*this->graphs[graphID])[v_].atGoal)
                {
                  goalConnectType gct;
                  gct.connected_to_existing_vertex = false;
                  gct.policy_end_vertex = v_;
                  goalConnections[graphID]= gct;
                  solutionFound = true;
                  break;
                }
              }

            }
            else
            {
              if ((*this->graphs[graphID])[newVertex].branchingFactor <= this->K_contingency_limit && !(*this->graphs[graphID])[newVertex].atGoal)
              {
                //check if we can connect to any belief on solution path
                for (int i_  = 0; i_ < vertexesOnPath.size(); i_++)
                {
                  Vertex vG0 = vertexesOnPath[i_];
                  this->model->setPosition(*(*this->graphs[mainGraph])[vG0].q);
                  this->model->updateFrames();

                  if (beliefConnectToBelief(newVertex, false, slidingNormal, vG0, graphID, mainGraph))
                  {
                    goalConnectType gct;
                    gct.connected_to_existing_vertex = true;
                    gct.existing_vertex = vG0;
                    Vertex end_ = copyVertex(vG0, mainGraph, graphID);

                    gct.policy_end_vertex = end_;
                    Edge e = this->addEdge(newVertex, end_, rl::plan::NoisyModel::CONNECT, (*this->graphs[graphID]), 1.0);
                    (*this->graphs[graphID])[e].chosenSampleDelta =
                        (*this->graphs[graphID])[end_].beliefState->configMean() -
                        (*this->graphs[graphID])[newVertex].beliefState->configMean();
                    (*this->graphs[graphID])[e].isBeliefConnectAction = true;
                    // node is on path
                    (*this->graphs[graphID])[newVertex].onSolutionPath = true;
                    (*this->graphs[graphID])[end_].onSolutionPath = true;
                    (*this->graphs[graphID])[end_].parent  = newVertex;
                    goalConnections[graphID]= gct;
                    solutionFound = true;
                    break;
                  }
                }
              }
            }
          }
        }
      }//end of loop through clusters -> all vertex & edge added
      return solutionFound;
    }//end of a well sempled results
  } // if sample failed
  else
  { // prune nodes that lead to local minimas and the planner gets stuck.
    if (n.first != leafVertexes_inLocalTree[graphID]){
      (*this->graphs[graphID])[n.first].failedExpand++;
      if ((*this->graphs[graphID])[n.first].failedExpand > 50)
        (*this->graphs[graphID])[n.first].exludeFromNN = true;
    }
  }

  return solutionFound;
}


bool SomaConcerrt::expandGraph(const ::std::vector<rl::plan::Particle>& particles,
                           ::rl::math::Vector3& slidingNormal,
                           Neighbor n,
                           rl::plan::NoisyModel::ActionType u,
                           ::rl::math::Transform goalT,
                           double reachability,
                           const int graphID,
                           goalConnectType& gct,
                           Vertex& newV,
                           const ::rl::math::Vector& chosen)
{
  ::boost::shared_ptr<rl::plan::BeliefState> belief = ::boost::make_shared<rl::plan::BeliefState>(particles, model, slidingNormal);
  rl::plan::Gaussian g = belief->configGaussian();

  // add a new vertex and edge
  rl::plan::VectorPtr mean = ::boost::make_shared<::rl::math::Vector>(g.mean);

  // if reachability is not 1.0 then a split happend, use it for visualisation of vertexes
  bool split = (reachability == 1.0)?false:true;

  Vertex newVertex = this->addVertex((*this->graphs[graphID]), mean, split);

  (*this->graphs[graphID])[newVertex].beliefState = belief;
  this->model->setPosition(g.mean);
  this->model->updateFrames();
  (*this->graphs[graphID])[newVertex].parent = n.first;
  (*this->graphs[graphID])[newVertex].hasSiblings = false;
  (*this->graphs[graphID])[newVertex].branchingFactor = (*this->graphs[graphID])[n.first].branchingFactor;
  newV = newVertex;

  // action type handling - get enum ID
  Edge e  = this->addEdge(n.first, newVertex, u, (*this->graphs[graphID]), reachability);
  // sampled relative joint conf motion on edge
  (*this->graphs[graphID])[e].chosenSampleDelta = chosen - (*this->graphs[graphID])[n.first].beliefState->configMean();

  //Check if new node is close to goal
  //double maxError = workSpaceDistance(belief, goalT);

  if (isAdmissableGoal((*this->graphs[graphID])[newVertex].beliefState) && (*this->graphs[graphID])[newVertex].branchingFactor < this->K_contingency_limit)
  {
    this->end.push_back(newVertex);
    (*this->graphs[graphID])[newVertex].atGoal = true;

    // save goal connect type related data
    gct.connected_to_existing_vertex = false;
    gct.policy_end_vertex = newVertex;
    return true;

  }
  // try Connect move to goal
  if (goalConnect(newVertex, true, slidingNormal, goalT, graphID))
  {

    // get all out edges of the node - find child at goal
    OutEdgeIteratorPair outEdgeIters = ::boost::out_edges(newVertex, (*this->graphs[graphID]));
    for (OutEdgeIterator edgeIt = outEdgeIters.first; edgeIt != outEdgeIters.second; ++edgeIt)
    {
      Vertex v_ = ::boost::target(*edgeIt, (*this->graphs[graphID]));
      if ((*this->graphs[graphID])[v_].atGoal)
      {
        gct.connected_to_existing_vertex = false;
        gct.policy_end_vertex = v_;
        break;
      }
    }
    return true;
  }

  if( (*this->graphs[graphID])[newVertex].branchingFactor < this->K_contingency_limit && !(*this->graphs[graphID])[newVertex].atGoal)
  {
    int mainGraph = 0;
    // loop through all possible beliefs and try merging with them
    //check if we can connect to any belief on solution path
    for (int i  = 0; i < vertexesOnPath.size(); i++)
    {
      Vertex vG0 = vertexesOnPath[i];
      this->model->setPosition(*(*this->graphs[mainGraph])[vG0].q);
      this->model->updateFrames();
      ::rl::math::Transform beliefGoalTransform = this->model->forwardPosition();

      double maxBeliefDistance = workSpaceDistance((*this->graphs[mainGraph])[newVertex].beliefState, beliefGoalTransform );

      if (beliefConnectToBelief(newVertex, false, slidingNormal, vG0, graphID, mainGraph))
      {

        gct.connected_to_existing_vertex = true;
        gct.existing_vertex = vG0;
        Vertex end_ = copyVertex(vG0, mainGraph, graphID);

        gct.policy_end_vertex = end_;
        Edge e = this->addEdge(newVertex, end_, rl::plan::NoisyModel::CONNECT, (*this->graphs[graphID]), 1.0);
        (*this->graphs[graphID])[e].chosenSampleDelta = (*this->graphs[graphID])[end_].beliefState->configMean() -
            (*this->graphs[graphID])[newVertex].beliefState->configMean();
        (*this->graphs[graphID])[e].isBeliefConnectAction = true;

        // node is on path
        (*this->graphs[graphID])[newVertex].onSolutionPath = true;
        (*this->graphs[graphID])[end_].onSolutionPath = true;
        (*this->graphs[graphID])[end_].parent  = newVertex;

        return true;
      }
    }
  }

  return false;
}
