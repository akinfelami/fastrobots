<p>Perform Grid localization for the sample trajectory.</p>
<ul>
  <li>You may modify the trajectory for better results.</li>
  <li>Attach a video of the best localization results (along the entire trajectory).</li>
</ul>

<h3 id="workflow-tips">Workflow Tips</h3>
<ul>
  <li>At all times, make sure only ONE instance of the plotter and simulator is running. Avoid running multiple simulation notebooks concurrently.</li>
  <li>If the plotter becomes sluggish over long operating times, restart it.</li>
  <li>There is a small “A” button that appears when you hover your mouse in the bottom left corner of the plotter tool. This fits the plot to the window.</li>
  <li>Use <TAB> for code completion in Jupyter Lab.</TAB></li>
  <li>There is a lot of information in the jupyter notebook. You can collapse a cell by clicking on the blue vertical bar to the left of cell, when you hover the mouse over it.</li>
  <li>Consider creating a copy of the base notebook and remove/collapse unnecessary cells.</li>
</ul>

<h3 id="implementation-tips">Implementation Tips</h3>
<p>Implementation in robotics can be a daunting task with multiple sub-objectives. It is always a good idea to list out the sub-objectives for your task and modularize your code accordingly. The Jupyter notebook provides you with a skeleton code and some helper functions. Test out some of these functions (<strong>to_map</strong>, <strong>from_map</strong>, <strong>normalize_angle</strong>) individually to get a hang of the larger codebase.</p>

<h4 id="useful-python-functions">Useful Python Functions</h4>
<p>Use the python module <strong>math</strong> to import various functions such as <strong>degrees</strong>, <strong>radians</strong>, <strong>cos</strong>, <strong>sin</strong>, <strong>tan2</strong>, <strong>hypot</strong>, etc. [Refer <a href="https://docs.python.org/3/library/math.html">math module functions</a>]</p>

<p>Use the python module <strong>numpy</strong> for Numpy operations such as basic slicing and indexing, functions such as <strong>sum()</strong>, <strong>cos</strong>, <strong>sin</strong>, <strong>tan2</strong>, <strong>arctan2</strong> and operators such as <strong>+</strong> and <strong>*</strong> . [Refer <a href="https://numpy.org/doc/stable/reference/routines.math.html">numpy math functions</a>]</p>

<p><strong>NOTE:</strong> Most of these functions use radians as the default unit.</p>

<h5 id="gaussian-function">Gaussian Function</h5>
<p>The <strong>gaussian</strong> function of class <strong>BaseLocalization</strong> can be used to model noise. It is advised to use Gaussians in the continuous world as it may be more involved to use Gaussians in a discretized world. Use the member functions <strong>from_map</strong> and <strong>to_map</strong> of class <strong>Mapper</strong> accordingly.</p>

<h5 id="arithmetic-underflow">Arithmetic Underflow</h5>
<p>When you multiply probabilities with each other (especially in the update step), the resulting value may suffer from <a href="https://en.wikipedia.org/wiki/Arithmetic_underflow">floating point underflow</a>. To prevent this from happening, normalize the belief grid when necessary using the following code:</p>
<div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>loc.bel = loc.bel / np.sum(loc.bel)
</code></pre></div></div>
<p>Think about how often you need to do this; every matrix operation takes time.</p>

<h5 id="odometry-motion-model-in-the-prediction-step">Odometry Motion Model in the Prediction Step</h5>
<p>In the odometry motion model, any control input <code class="language-plaintext highlighter-rouge">u</code> can be expressed as a tuple <span><img src="https://latex.codecogs.com/svg.latex?u%20%5Cequiv%20%5Brot1,%20trans,%20rot2%5D" /></span></p>

<p>The <code class="language-plaintext highlighter-rouge">compute_control</code> function is expected to extract the control information in the above format given a previous pose (at time <code class="language-plaintext highlighter-rouge">t-1</code>) and a current pose (at time <code class="language-plaintext highlighter-rouge">t</code>) of the robot.</p>

<p><code class="language-plaintext highlighter-rouge">actual_u = compute_control(cur_odom, prev_odom)</code>, where <code class="language-plaintext highlighter-rouge">cur_odom</code> and <code class="language-plaintext highlighter-rouge">prev_odom</code> are the actual odometry readings extracted from the robot sensors before and after the robot motion at time <code class="language-plaintext highlighter-rouge">t</code>. Although this will be a noisy estimate of <code class="language-plaintext highlighter-rouge">u</code>, it is better than no prior at all!</p>

<p>Now that you have the <code class="language-plaintext highlighter-rouge">actual_u</code>, you need to incorporate the effects of this motion into the belief of the robot (from time <code class="language-plaintext highlighter-rouge">t-1</code>). You can compute what is the “necessary control action” <code class="language-plaintext highlighter-rouge">u</code> required for some arbitrary set of previous pose and current pose using the same function i.e <code class="language-plaintext highlighter-rouge">compute_control(cur_pose, prev_pose)</code>.</p>

<p>Now, we have the <code class="language-plaintext highlighter-rouge">actual_u</code> and a <code class="language-plaintext highlighter-rouge">u</code> for a pair of possible previous and current poses of the robot. We can “plug” these into a Gaussian function to see how “probable” is the transition of the robot state from <code class="language-plaintext highlighter-rouge">prev_pose</code> to <code class="language-plaintext highlighter-rouge">current_pose</code> given the actual control action is <code class="language-plaintext highlighter-rouge">actual_u</code>.</p>

<p align="center"><img src="../Figs/odom_equation.gif" width="450" /></p>

<p>where, <code class="language-plaintext highlighter-rouge">gaussian</code> is a function defined in <strong>localization.py</strong>, <code class="language-plaintext highlighter-rouge">odom_rot_noise</code> and <code class="language-plaintext highlighter-rouge">odom_trans_noise</code> are variables defined in <strong>localization.py</strong>, and <span><img src="https://latex.codecogs.com/svg.latex?p({x}_{t}|{u}_{t},{x}_{t-1})" /></span> is the transitional probability of a robot from a previous state <span><img src="https://latex.codecogs.com/svg.latex?{x}_{t-1}" /></span> to the current state <span><img src="https://latex.codecogs.com/svg.latex?{x}_{t}" /></span> given the current input <span><img src="https://latex.codecogs.com/svg.latex?{u}_{t}" /></span>. The above equation is essentially what the function <code class="language-plaintext highlighter-rouge">odom_motion_model(cur_pose, prev_pose, u)</code> should implement.</p>

<p>All the quantities in the RHS (Right Hand Side) of the above equation is known, and thus you can calculate the transition probability <span><img src="https://latex.codecogs.com/svg.latex?p({x}_{t}|{u}_{t},{x}_{t-1})" /></span> which is the only unknown quantity in the prediction step of the Bayes Filter. Repeat this for every possible pair of previous and current poses (the first line of the Bayes Filter algorithm) to complete the prediction step.</p>

<h5 id="sensor-model-with-multiple-individual-measurements">Sensor Model with multiple individual measurements</h5>
<p>Each measurement <img src="https://latex.codecogs.com/svg.latex?z_{t}" /> consists of 18 different individual measurements <img src="https://latex.codecogs.com/svg.latex?z^{1}_{t}, z^{2}_{t}, ...., z^{18}_{t}" /> recorded at equidistant angular positions during the robot’s (anti-clockwise) rotation behavior. The 18 true measurements values are recorded at the same equidistant angular positions for each grid cell (state) and is available through the <strong>Mapper</strong> class. Therefore, each index of the member variable array <strong>obs_views</strong> (of class <strong>Mapper</strong>) is the true individual measurement of the corresponding index of the member variable array <strong>obs_range_data</strong> (of class <strong>BaseLocalization</strong>). You do not need to use <strong>obs_bearing_data</strong> in your Bayes Filter implementation.</p>

<p>You will need to find the likelihood of the 18 measurements given a state i.e.</p>

<p align="center"><img src="https://latex.codecogs.com/svg.latex?p(z_{t}|x_{t},m) = \prod_{k=1}^{18} p(z^{k}_{t}|x_{t}, m)" width="300" /></p>

<p>In the above equation, we assume that individual measurements are independent given the robot state.</p>

<h5 id="normalizing-your-angles">Normalizing your angles</h5>
<p>The third dimension of the grid represents the orientation (yaw) in the range [-180,+180) degrees. When dealing with angles in your bayes filter (for example in calculating rotation1 and rotation2 in the odom motion model), you need to make sure the final angles are in the above range.</p>

<p>Think about what happens when you use a Gaussian to model a rotation of 350 degrees where the true value is -10 degrees and standard deviation is 20 degrees. <img src="https://latex.codecogs.com/svg.latex?\mathcal{N}(350|\mu=-10,\sigma=20)" /> is highly unlikely though 350 degrees is equivalent to -10 degrees.</p>

<p>Use the function <strong>normalize_angle</strong> from class <strong>Mapper</strong> to normalize your angles when necessary.</p>

<h5 id="computation-time">Computation Time</h5>
<p>In each iteration of the bayes filter, you will need to go through all possible previous and current states in the grid to estimate the belief. Think about how many loops would be required to perform this.</p>

<p>Given that your grid has 12×9×18 = 1944 possible states, there is lot of computation involved in each iteration of the bayes filter. Hence, you need to write efficient code, especially in Python, if you want your entire estimation process to run within a couple of minutes. Try to get the running time of the prediction step and the update step functions to be within a couple of seconds; shorter running times may prove beneficial for testing and debugging. Here are some ways to reduce processing time:</p>
<ul>
  <li>If the probability of a state (i.e grid cell) is 0, then we can skip that grid cell in the inner loops of the prediction step of the bayes filter. (i.e only in multiplicative terms, since multiplying any value with a 0 results in a 0). In fact, if a state has a probability less than <code class="language-plaintext highlighter-rouge">0.0001</code>, we can skip those states as they don’t contribute a lot to the belief and thus can reduce the computation time. Note that since you are skipping some states, the sum of the probabilities across the grid may no longer sum to 1. So you will need to make sure you normalize at the end of your prediction step and update step (which you already do as per the algorithm).</li>
  <li>Reduce unnecessary interim variables. This can lead to slow processing times, especially in Python.</li>
  <li>Use Numpy for faster matrix operations instead of element wise operations. Numpy is faster if you can use matrix-like operations because the backend processing happens in C.
    <ul>
      <li><strong>HINT</strong>: The <strong>gaussian</strong> function of class BaseLocalization can handle Numpy variables; think about how to perform Numpy operations in the update step. You may not even need the <strong>sensor_model</strong> function if you perform grid indexing to directly compute the sensor noise model in the <strong>update_step</strong> function.</li>
    </ul>
  </li>
