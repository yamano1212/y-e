import numpy as np
import gym
from gym import wrappers

class QL(object):
	"""
	Q-Learning algorithm.
	"""
	def __init__(self, env, alpha, gamma, epsilon, bins=40, ema_coeff=0.01):
		self.env = env
		self.alpha = alpha
		self.gamma = gamma
		self.epsilon = epsilon
		self.ndim_obs = self.env.observation_space.shape[0]
		self.n_actions = self.env.action_space.n

		self.bins = bins
		max_integerized_obs = self.to_integer(self.env.observation_space.high)
		self.q_function = np.zeros((max_integerized_obs, self.n_actions))

		self.episode_count = 0
		self.average_episode_step = None
		self.ema_coeff = ema_coeff

	def to_integer(self, obs):
		discretized = np.zeros(self.ndim_obs)
		for i in range(self.ndim_obs):
			low = self.env.observation_space.low[i]
			high = self.env.observation_space.high[i]
			discretized[i] = int((obs[i] - low) / (high - low) * self.bins)
			if discretized[i] == self.bins:
				discretized[i] -= 1
		integerized = np.sum((self.bins ** np.arange(self.ndim_obs)) * discretized).astype(np.int)
		return integerized

	def act(self, obs):
		if np.random.uniform(0, 1) > self.epsilon:
			return self.act_greedily(obs)
		else:
			return np.random.choice(self.n_actions)

	def act_greedily(self, obs):
		state = self.to_integer(obs)
		return np.argmax(self.q_function[state, :])

	def update(self, obs, action, reward, next_obs, is_terminal):
		state = self.to_integer(obs)
		next_state = self.to_integer(next_obs)
		q = self.q_function[state, action]
		target = reward
		if not is_terminal:
			target += self.gamma * self.q_function[next_state,:].max()
		self.q_function[state, action] = self.alpha * target + (1 - self.alpha) * q

	def run_episode(self, train):
		step_count = 0
		obs = self.env.reset()
		done = False
		while not done:
			step_count += 1

			if train:
				action = self.act(obs)
				next_obs, reward, done, info = self.env.step(action)
				self.update(obs, action, reward, next_obs, done)
			else:
				action = self.act_greedily(obs)
				next_obs, reward, done, info = self.env.step(action)
			obs = next_obs

		self.episode_count += 1
		if self.average_episode_step is None:
			self.average_episode_step = step_count
		else:
			self.average_episode_step *= (1 - self.ema_coeff)
			self.average_episode_step += self.ema_coeff * step_count

	def run(self, episode, result_callable=None):
		if result_callable is None:
			result_callable = lambda x: False
		for _ in range(episode):
			if result_callable(self.episode_count):
				self.run_episode(train=False)
				self.display_statistics()
			else:
				self.run_episode(train=True)
		self.env.close()

	def display_statistics(self):
		print('===statistics===')
		print('episode_count', self.episode_count)
		print('average_episode_step', self.average_episode_step)

if __name__ == '__main__':
	env = gym.make('MountainCar-v0')
	frequency = lambda x: (x + 1) % 500 == 0
	env = wrappers.Monitor(env, directory='movies', force=True, video_callable=frequency)
	model = QL(env, alpha=0.1, gamma=0.99, epsilon=0.1, bins=40)
	model.run(100000, result_callable=frequency)
	env.env.close()
