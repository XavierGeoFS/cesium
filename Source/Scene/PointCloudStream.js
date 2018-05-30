define([
        '../Core/arraySlice',
        '../Core/Cartesian2',
        '../Core/Cartesian3',
        '../Core/Cartesian4',
        '../Core/Color',
        '../Core/combine',
        '../Core/ComponentDatatype',
        '../Core/defaultValue',
        '../Core/defined',
        '../Core/defineProperties',
        '../Core/destroyObject',
        '../Core/DeveloperError',
        '../Core/FeatureDetection',
        '../Core/getStringFromTypedArray',
        '../Core/JulianDate',
        '../Core/Math',
        '../Core/Matrix3',
        '../Core/Matrix4',
        '../Core/oneTimeWarning',
        '../Core/OrthographicFrustum',
        '../Core/Plane',
        '../Core/PrimitiveType',
        '../Core/RuntimeError',
        '../Core/TaskProcessor',
        '../Core/Transforms',
        '../Renderer/Buffer',
        '../Renderer/BufferUsage',
        '../Renderer/DrawCommand',
        '../Renderer/Pass',
        '../Renderer/RenderState',
        '../Renderer/ShaderProgram',
        '../Renderer/ShaderSource',
        '../Renderer/VertexArray',
        '../ThirdParty/when',
        './BlendingState',
        './Cesium3DTileBatchTable',
        './Cesium3DTileFeature',
        './Cesium3DTileFeatureTable',
        './ClippingPlaneCollection',
        './getClipAndStyleCode',
        './getClippingFunction',
        './PointCloud',
        './PointCloudEyeDomeLighting',
        './PointCloudShading',
        './SceneMode',
        './ShadowMode'
    ], function(
        arraySlice,
        Cartesian2,
        Cartesian3,
        Cartesian4,
        Color,
        combine,
        ComponentDatatype,
        defaultValue,
        defined,
        defineProperties,
        destroyObject,
        DeveloperError,
        FeatureDetection,
        getStringFromTypedArray,
        JulianDate,
        CesiumMath,
        Matrix3,
        Matrix4,
        oneTimeWarning,
        OrthographicFrustum,
        Plane,
        PrimitiveType,
        RuntimeError,
        TaskProcessor,
        Transforms,
        Buffer,
        BufferUsage,
        DrawCommand,
        Pass,
        RenderState,
        ShaderProgram,
        ShaderSource,
        VertexArray,
        when,
        BlendingState,
        Cesium3DTileBatchTable,
        Cesium3DTileFeature,
        Cesium3DTileFeatureTable,
        ClippingPlaneCollection,
        getClipAndStyleCode,
        getClippingFunction,
        PointCloud,
        PointCloudEyeDomeLighting,
        PointCloudShading,
        SceneMode,
        ShadowMode) {
    'use strict';

    /**
     * A method for streaming time-dynamic point cloud data.
     *
     * @alias PointCloudStream
     * @constructor
     *
     * @private
     */
    function PointCloudStream(options) {
        this.pointCloudShading = new PointCloudShading(options.pointCloudShading);
        this.style = options.style;
        this.clippingPlanes = options.clippingPlanes; // TODO : getter/setter for ownership
        this.shadows = defaultValue(options.shadows, ShadowMode.ENABLED);

        this.modelMatrix = Matrix4.clone(Matrix4.IDENTITY);
        this.show = true;
        this.index = 0;

        this._pointCloudEyeDomeLighting = new PointCloudEyeDomeLighting();
        this._loadTimestamp = undefined;
        this._clippingPlanesState = 0;
        this._pickId = undefined;
        this._frames = [];
    }

    function getPickFragmentShaderLoaded(stream) {
        return function(fs) {
            return ShaderSource.createPickFragmentShaderSource(fs, 'uniform');
        };
    }

    function getPickUniformMapLoaded(stream) {
        return function(uniformMap) {
            return combine(uniformMap, {
                czm_pickColor : function() {
                    return stream._pickId.color;
                }
            });
        };
    }

    PointCloudStream.prototype.addFrame = function(index, arrayBuffer) {
        this._frames[index] = new PointCloud({
            arrayBuffer : arrayBuffer,
            pickFragmentShaderLoaded : getPickFragmentShaderLoaded(this),
            pickUniformMapLoaded : getPickUniformMapLoaded(this)
        });
    };

    PointCloudStream.prototype.update = function(frameState) {
        if (frameState.mode === SceneMode.MORPHING) {
            return;
        }

        if (!this.show) {
            return;
        }

        var frames = this._frames;
        var framesLength = frames.length;
        if (framesLength === 0) {
            return;
        }

        var index = this.index;
        if (index < 0 || index > framesLength) {
            // Index not in range.
            return;
        }

        var frame = frames[index];
        if (!defined(frame)) {
            return;
        }

        if (!defined(this._pickId)) {
            this._pickId = frameState.context.createPickId({
                primitive : this
            });
        }

        if (!defined(this._loadTimestamp)) {
            this._loadTimestamp = JulianDate.clone(frameState.time);
        }

        var timeSinceLoad = Math.max(JulianDate.secondsDifference(frameState.time, this._loadTimestamp) * 1000, 0.0);

        // Update clipping planes
        var clippingPlanes = this.clippingPlanes;
        var clippingPlanesState = 0;
        var clippingPlanesDirty = false;
        var isClipped = defined(clippingPlanes) && clippingPlanes.enabled;

        if (isClipped) {
            clippingPlanes.update(frameState);
            clippingPlanesState = clippingPlanes.clippingPlanesState;
        }

        if (this._clippingPlanesState !== clippingPlanesState) {
            this._clippingPlanesState = clippingPlanesState;
            clippingPlanesDirty = true;
        }

        var pointCloudShading = this.pointCloudShading;
        var eyeDomeLighting = this._pointCloudEyeDomeLighting;

        var commandList = frameState.commandList;
        var lengthBeforeUpdate = commandList.length;

        if (defined(frame)) {
            frame.style = this.style;
            frame.modelMatrix = this.modelMatrix; // TODO : there may also be a per-frame model matrix that we need to deal with
            frame.time = timeSinceLoad;
            frame.shadows = this.shadows;
            frame.clippingPlanes = clippingPlanes;
            frame.isClipped = isClipped;
            frame.clippingPlanesDirty = clippingPlanesDirty;
            frame.attenuation = pointCloudShading.attenuation;
            frame.geometricError = 0.0; // TODO : If we had a bounding volume we could derive it
            frame.geometricErrorScale = pointCloudShading.geometricErrorScale;
            frame.maximumAttenuation = defined(pointCloudShading.maximumAttenuation) ? pointCloudShading.maximumAttenuation : 10;
            frame.update(frameState);
        }

        var lengthAfterUpdate = commandList.length;
        var addedCommandsLength = lengthAfterUpdate - lengthBeforeUpdate;

        if (pointCloudShading.attenuation && pointCloudShading.eyeDomeLighting && (addedCommandsLength > 0)) {
            eyeDomeLighting.update(frameState, lengthBeforeUpdate, pointCloudShading);
        }
    };

    PointCloudStream.prototype.isDestroyed = function() {
        return false;
    };

    PointCloudStream.prototype.destroy = function() {
        var frames = this._frames;
        var framesLength = frames.length;
        for (var i = 0; i < framesLength; ++i) {
            var frame = frames[i];
            if (defined(frame)) {
                frame.destroy();
            }
        }
        this._frames = undefined;
        return destroyObject(this);
    };

    return PointCloudStream;
});
