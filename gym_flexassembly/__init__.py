# # import gym
# # from gym.envs.registration import registry, make, spec
# from gym.envs.registration import register

# # def register(id, *args, **kvargs):
# #   if id in registry.env_specs:
# #     return
# #   else:
# #     return gym.envs.registration.register(id, *args, **kvargs)
 
# register(id='FlexAssembly-v0', 
#     entry_point='gym_flexassembly.envs:FlexAssemblyEnv',
#     # max_episode_steps=1000,
#     # reward_threshold=5.0,
#     # kwargs={'render': True}
# )

# # def getList():
# #     btenvs = ['- ' + spec.id for spec in gym.envs.registry.all() if spec.id.find('Bullet') >= 0]
# #     return btenvs