import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

def small_unet(input_shape):
    inputs = keras.Input(shape=input_shape)
    x1 = layers.Conv2D(16, 3, padding='same', activation='relu')(inputs)
    x1 = layers.Conv2D(16, 3, padding='same', activation='relu')(x1)
    p1 = layers.MaxPool2D()(x1)
    x2 = layers.Conv2D(32, 3, padding='same', activation='relu')(p1)
    x2 = layers.Conv2D(32, 3, padding='same', activation='relu')(x2)
    p2 = layers.MaxPool2D()(x2)
    b  = layers.Conv2D(64, 3, padding='same', activation='relu')(p2)
    u1 = layers.UpSampling2D()(b)
    c1 = layers.Concatenate()([u1, x2])
    x3 = layers.Conv2D(32, 3, padding='same', activation='relu')(c1)
    u2 = layers.UpSampling2D()(x3)
    c2 = layers.Concatenate()([u2, x1])
    x4 = layers.Conv2D(16, 3, padding='same', activation='relu')(c2)
    out = layers.Conv2D(1, 1, activation='sigmoid')(x4) # 0..1 traversability
    return keras.Model(inputs, out)
