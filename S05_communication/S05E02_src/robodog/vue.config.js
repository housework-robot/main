const {
    defineConfig
} = require('@vue/cli-service')
const CopyWebpackPlugin = require('copy-webpack-plugin');
module.exports = defineConfig({
    transpileDependencies: true,
    // easy-player  相关
    configureWebpack: {
        plugins: [
            new CopyWebpackPlugin([{
                    from: 'node_modules/@easydarwin/easyplayer/dist/component/EasyPlayer.swf',
                    to: './libs/EasyPlayer/'
                },
                {
                    from: 'node_modules/@easydarwin/easyplayer/dist/component/crossdomain.xml',
                    to: './libs/EasyPlayer/'
                },
                {
                    from: 'node_modules/@easydarwin/easyplayer/dist/component/EasyPlayer-lib.min.js',
                    to: './libs/EasyPlayer/'
                }
            ]),
        ]
    },
    devServer: {
        proxy: {
            '/easyPlayerPro': {
                target: 'webrtc://global-lebtest-play.myqcloud.com', // 你的API服务器地址
                changeOrigin: true, // 必须设置为true，才能代理到不同的域名
                secure: false,
                pathRewrite: {
                    '^/easyPlayerPro': '' // 重写路径：去掉路径中的/api部分
                }
            },
            '/live222': {
                target: 'rtsp://127.0.0.1:10054/live', // 你的API服务器地址
                changeOrigin: true, // 必须设置为true，才能代理到不同的域名
                secure: false,
                pathRewrite: {
                    '^/live': '' // 重写路径：去掉路径中的/api部分
                }
            },
        }
    },
    transpileDependencies: true,
    lintOnSave: false,
    productionSourceMap: false
});
