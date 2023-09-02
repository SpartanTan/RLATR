from gymnasium.envs.registration import register

def register_factory_envs():
     register(
          id="training-factory-v0",
          entry_point='factory_env.envs:SimFactoryEnv',
     )

     register(
          id="real-factory-v0",
          entry_point='factory_env.envs:FactoryEnv',
     )       