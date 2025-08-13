const CACHE_NAME = 'drone-panel-static-v1';
const ASSETS = [
  '/',
  '/static/css/bootstrap.min.css',
  '/static/leaflet/leaflet.css',
  '/static/js/chart.umd.min.js',
  '/static/js/date-fns.min.js',
  '/static/js/chartjs-adapter-date-fns.umd.min.js',
  '/static/js/leaflet.js',
  '/static/js/sw-register.js'
];

self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE_NAME).then(cache => cache.addAll(ASSETS)));
});
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then(keys => Promise.all(keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k))))
  );
});
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  if (ASSETS.some(p => url.pathname === p)) {
    event.respondWith(caches.match(request).then(c => c || fetch(request)));
    return;
  }
  if (url.pathname.startsWith('/api/')) return;

  if (request.mode === 'navigate') {
    event.respondWith(fetch(request).catch(() => caches.match('/')));
  }
});
