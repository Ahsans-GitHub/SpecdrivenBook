module.exports = function (context, options) {
  return {
    name: 'floating-chatbot-plugin',
    getClientModules() {
      return [require.resolve('./client-module.tsx')];
    },
  };
};