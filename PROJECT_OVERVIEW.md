# WebGL Fluid Simulation - Project Overview

## Overview
This is a real-time fluid dynamics simulation implemented using WebGL, designed to work on both desktop and mobile browsers. The simulation uses GPU-accelerated compute shaders to achieve high-performance fluid rendering with interactive controls and camera integration.

## Key Features

### Core Simulation
- **Real-time Fluid Dynamics**: GPU-accelerated fluid simulation using WebGL shaders
- **Configurable Physics**: Adjustable density dissipation, velocity dissipation, pressure, and curl parameters
- **Multi-resolution Support**: Separate resolutions for simulation (256px), dye rendering (1024px), and capture (512px)
- **Mobile Optimization**: Automatic resolution adjustments for mobile devices

### Visual Effects
- **Bloom Effect**: Multi-pass bloom with configurable intensity, threshold, and soft knee
- **Sunrays**: Volumetric lighting effect with weight-based intensity
- **Dynamic Coloring**: Color-changing fluid with adjustable update speed
- **Shading**: Optional 3D-style shading for enhanced visual depth
- **Transparency**: Optional transparent background support

### Camera Integration
- **Motion Detection**: Real-time camera motion analysis using optical flow
- **Depth Sensitivity**: Motion intensity affects fluid generation strength
- **Camera Smoothing**: Smoothed motion tracking to prevent jittery interactions
- **Drag Simulation**: Camera motion translates to fluid drag forces

### Interactive Controls
- **Touch/Mouse Input**: Multi-touch support for fluid interaction
- **GUI Controls**: Real-time parameter adjustment using dat.GUI
- **Fullscreen Support**: Click upper-left corner to enter fullscreen
- **GUI Toggle**: Hide/show controls interface
- **Screenshot Capture**: Built-in screenshot functionality

## Technical Implementation

### WebGL Architecture
- **WebGL 2.0/1.0 Support**: Automatic fallback to WebGL 1.0 if 2.0 unavailable
- **Half-float Textures**: Efficient GPU memory usage with half-precision floating point
- **Format Detection**: Automatic detection of supported texture formats
- **Multi-buffer Rendering**: Separate framebuffers for different simulation stages

### Shader Programs
The simulation uses multiple shader programs for different stages:
- **Advection**: Fluid velocity and dye advection
- **Pressure**: Pressure field calculation with iterative solving
- **Divergence**: Velocity divergence computation
- **Curl**: Vorticity calculation for fluid turbulence
- **Splat**: Adding forces and dye to the simulation
- **Display**: Final rendering with bloom and sunrays

### Configuration Parameters

#### Simulation Settings
- `SIM_RESOLUTION`: Base simulation resolution (256)
- `DYE_RESOLUTION`: Dye rendering resolution (1024)
- `PRESSURE_ITERATIONS`: Pressure solver iterations (20)
- `CURL`: Vorticity strength (30)
- `SPLAT_RADIUS`: Input force radius (0.25)
- `SPLAT_FORCE`: Input force strength (6000)

#### Visual Settings
- `BLOOM_ITERATIONS`: Bloom pass count (8)
- `BLOOM_INTENSITY`: Bloom effect strength (0.8)
- `SUNRAYS_WEIGHT`: Volumetric lighting intensity (1.0)
- `COLOR_UPDATE_SPEED`: Color transition speed (10)

#### Camera Settings
- `MOTION_SENSITIVITY`: Camera motion response (0.5)
- `MOTION_THRESHOLD`: Motion detection threshold (0.02)
- `CAMERA_SMOOTHING`: Motion smoothing factor (0.3)
- `DEPTH_SENSITIVITY`: Depth-based force scaling (2.0)

## File Structure

### Core Files
- `script.js` (62KB): Main simulation engine and WebGL implementation
- `index.html`: HTML page with canvas setup and styling
- `dat.gui.min.js`: GUI controls library

### Assets
- `logo.png`: Application logo
- `screenshot.jpg`: Demo screenshot
- `LDR_LLL1_0.png`: Environment texture
- `iconfont.ttf`: Custom icon font for UI
- `app_badge.png`, `gp_badge.png`: Platform badges

### Configuration
- `LICENSE`: MIT license
- `README.md`: Basic project documentation
- `.github/FUNDING.yml`: GitHub funding configuration

## Browser Compatibility
- **Desktop**: All modern browsers supporting WebGL
- **Mobile**: iOS Safari, Android Chrome with touch interaction
- **WebGL Fallback**: Automatic quality reduction for limited hardware
- **Format Support**: Automatic texture format detection and fallback

## Performance Optimizations
- **Mobile Detection**: Automatic resolution scaling for mobile devices
- **Feature Detection**: Disables effects on unsupported hardware
- **Efficient Rendering**: Multi-pass rendering with minimal texture copying
- **Memory Management**: Proper cleanup of WebGL resources
- **Camera Integration**: Optional camera processing to preserve performance

## Usage
1. Open `index.html` in a WebGL-supported browser
2. Click and drag to interact with the fluid
3. Use the GUI (top-right) to adjust parameters
4. Click upper-left corner to toggle fullscreen/GUI
5. Enable camera motion for hands-free interaction

## Technical References
- NVIDIA GPU Gems: Fast Fluid Dynamics Simulation on GPU
- Real-time fluid simulation techniques
- WebGL shader programming best practices
- Mobile browser optimization strategies