/*
 * Dependency map.
 * Nodes are QoS policies, edges are rules. Colours encode the consequence:
 *   red   = won't connect        (Structural)
 *   orange= breaks a guarantee   (Functional)
 *   slate = wastes resources     (Operational)
 *
 * Interaction:
 *   - Click a connection (edge) to ISOLATE that single rule: only its two
 *     policies and the link between them stay visible. A side panel shows the
 *     plain title, the consequence, and a link to the rule page.
 *   - Click a policy (node) to see just that policy's conflicts.
 *   - "Back to map" (or clicking empty space) restores the full graph.
 */
(function () {
  'use strict';

  var TIER_COLORS = {
    Structural: '#A61E1E',
    Functional: '#E8590C',
    Operational: '#475569'
  };

  var NODE_POSITIONS = {
    HIST: { x: 160, y: 120 }, RELIAB: { x: 360, y: 120 }, DURABL: { x: 560, y: 120 }, LIVENS: { x: 760, y: 120 },
    RESLIM: { x: 160, y: 300 }, LFSPAN: { x: 360, y: 300 }, DEADLN: { x: 560, y: 300 }, DESTORD: { x: 760, y: 300 },
    PART: { x: 160, y: 480 }, OWNST: { x: 360, y: 480 }, WDLIFE: { x: 560, y: 480 }, RDLIFE: { x: 760, y: 480 },
    ENTFAC: { x: 160, y: 650 }, USRDATA: { x: 360, y: 650 }, GRPDATA: { x: 560, y: 650 }, TOPDATA: { x: 760, y: 650 }
  };

  var FIT_PADDING = 56;

  var NODE_HREFS = {
    ENTFAC: '../qos/entity-factory/', PART: '../qos/partition/', USRDATA: '../qos/user-data/',
    GRPDATA: '../qos/group-data/', TOPDATA: '../qos/topic-data/', RELIAB: '../qos/reliability/',
    DURABL: '../qos/durability/', DEADLN: '../qos/deadline/', LIVENS: '../qos/liveliness/',
    HIST: '../qos/history/', RESLIM: '../qos/resource-limits/', LFSPAN: '../qos/lifespan/',
    OWNST: '../qos/ownership/', DESTORD: '../qos/destination-order/', WDLIFE: '../qos/writer-data-lifecycle/',
    RDLIFE: '../qos/reader-data-lifecycle/'
  };

  var cyInstance = null;
  var resizeHandler = null;
  var resizeTimer = null;
  var panel = null;
  var isolated = false;

  function tierColor(dep) { return TIER_COLORS[dep] || '#64748b'; }

  function fitAll() {
    if (!cyInstance) { return; }
    cyInstance.resize();
    cyInstance.fit(undefined, FIT_PADDING);
    cyInstance.center();
  }

  function fitTo(collection) {
    if (!cyInstance) { return; }
    cyInstance.resize();
    cyInstance.fit(collection, 90);
  }

  function scheduleFit() {
    if (resizeTimer) { window.clearTimeout(resizeTimer); }
    resizeTimer = window.setTimeout(function () {
      if (!isolated) { fitAll(); }
    }, 120);
  }

  function destroyGraph() {
    isolated = false;
    if (resizeTimer) { window.clearTimeout(resizeTimer); resizeTimer = null; }
    if (resizeHandler) { window.removeEventListener('resize', resizeHandler); resizeHandler = null; }
    if (cyInstance) { cyInstance.destroy(); cyInstance = null; }
    if (panel) { panel.remove(); panel = null; }
  }

  function consequenceChip(type, text) {
    return '<span class="rule-consequence rule-consequence-' + type + '">' + text + '</span>';
  }

  function ensurePanel(container) {
    panel = document.createElement('div');
    panel.className = 'depgraph-panel';
    panel.hidden = true;
    panel.innerHTML =
      '<button type="button" class="depgraph-back">&larr; Back to map</button>' +
      '<div class="depgraph-panel-body"></div>';
    container.appendChild(panel);
    panel.querySelector('.depgraph-back').addEventListener('click', function () {
      restore();
    });
  }

  function showPanel(html) {
    if (!panel) { return; }
    panel.querySelector('.depgraph-panel-body').innerHTML = html;
    panel.hidden = false;
  }

  function hidePanel() {
    if (panel) { panel.hidden = true; }
  }

  function showOnly(keep) {
    if (!cyInstance) { return; }
    isolated = true;
    cyInstance.batch(function () {
      cyInstance.elements().style('display', 'none');
      keep.style('display', 'element');
    });
    fitTo(keep);
  }

  function restore() {
    if (!cyInstance) { return; }
    isolated = false;
    cyInstance.batch(function () {
      cyInstance.elements().style('display', 'element');
    });
    hidePanel();
    fitAll();
  }

  /* Isolate one rule: its edge plus the two endpoint policies only. */
  function isolateRule(edge) {
    var nodes = edge.connectedNodes();
    var keep = edge.union(nodes);
    showOnly(keep);
    var d = edge.data();
    var src = cyInstance.getElementById(d.source).data('fullName') || d.source;
    var tgt = cyInstance.getElementById(d.target).data('fullName') || d.target;
    var pair = (d.source === d.target)
      ? src + ' with itself'
      : src + ' &rarr; ' + tgt;
    showPanel(
      '<p class="depgraph-panel-kicker">Rule ' + d.rule + '</p>' +
      '<h3 class="depgraph-panel-title">' + d.title + '</h3>' +
      '<p class="depgraph-panel-pair">' + pair + '</p>' +
      consequenceChip(d.consequenceType, d.consequence) +
      '<p><a class="depgraph-panel-link" href="' + d.href + '">Open rule page &rarr;</a></p>'
    );
  }

  /* Focus one policy: the node, its edges, and its neighbours. */
  function focusPolicy(node) {
    var edges = node.connectedEdges();
    var keep = node.union(edges).union(edges.connectedNodes());
    showOnly(keep);
    var name = node.data('fullName') || node.id();
    var items = '';
    var seen = {};
    edges.forEach(function (edge) {
      var d = edge.data();
      if (seen[d.rule]) { return; }
      seen[d.rule] = true;
      items +=
        '<a class="depgraph-conflict" href="' + d.href + '">' +
        '<span class="depgraph-conflict-title">' + d.title + '</span>' +
        consequenceChip(d.consequenceType, d.consequence) +
        '</a>';
    });
    var count = Object.keys(seen).length;
    var pageHref = NODE_HREFS[node.id()];
    var pageLink = pageHref
      ? '<p><a class="depgraph-panel-link" href="' + pageHref + '">Open ' + name + ' page &rarr;</a></p>'
      : '';
    showPanel(
      '<p class="depgraph-panel-kicker">Policy</p>' +
      '<h3 class="depgraph-panel-title">' + name + '</h3>' +
      '<p class="depgraph-panel-pair">' + count + (count === 1 ? ' conflict' : ' conflicts') + '</p>' +
      '<div class="depgraph-conflict-list">' + (items || '<em>No conflicts.</em>') + '</div>' +
      pageLink
    );
  }

  function buildElements(data) {
    var nodes = (data.nodes || []).map(function (node) {
      var pos = NODE_POSITIONS[node.id] || { x: 0, y: 0 };
      return { data: { id: node.id, label: node.id, fullName: node.fullName }, position: { x: pos.x, y: pos.y } };
    });
    var edges = (data.edges || []).map(function (edge) {
      return {
        data: {
          id: edge.id, rule: edge.rule, source: edge.source, target: edge.target,
          label: String(edge.rule), href: edge.href, dependency: edge.dependency, arrow: edge.arrow,
          title: edge.title, consequence: edge.consequence, consequenceType: edge.consequenceType
        }
      };
    });
    return nodes.concat(edges);
  }

  function renderGraph(data) {
    var container = document.getElementById('dependency-graph');
    if (!container || typeof cytoscape === 'undefined') { return; }
    destroyGraph();
    ensurePanel(container);

    cyInstance = cytoscape({
      container: container,
      elements: buildElements(data),
      minZoom: 0.25,
      maxZoom: 2.5,
      wheelSensitivity: 0.2,
      boxSelectionEnabled: false,
      selectionType: 'single',
      style: [
        {
          selector: 'node',
          style: {
            'shape': 'round-rectangle', 'label': 'data(label)', 'text-valign': 'center', 'text-halign': 'center',
            'font-size': '14px', 'font-family': 'Roboto, system-ui, -apple-system, sans-serif', 'font-weight': 700,
            'color': '#253044', 'background-color': '#f8fbff', 'border-width': 1.4, 'border-color': '#6f82d8',
            'width': 112, 'height': 44, 'text-wrap': 'wrap', 'text-max-width': 100,
            'shadow-blur': 10, 'shadow-color': '#1e293b', 'shadow-opacity': 0.08, 'shadow-offset-y': 3, 'z-index': 20,
            'transition-property': 'background-color, border-color, border-width, opacity', 'transition-duration': '0.15s'
          }
        },
        { selector: 'node.hover', style: { 'background-color': '#eef4ff', 'border-color': '#4e63d6', 'border-width': 2, 'z-index': 30 } },
        { selector: 'node.selected', style: { 'background-color': '#e8efff', 'border-color': '#3346b8', 'border-width': 2.6, 'z-index': 40 } },
        { selector: 'node.dimmed', style: { 'opacity': 0.22 } },
        {
          selector: 'edge',
          style: {
            'width': 1.8, 'curve-style': 'bezier', 'control-point-step-size': 80, 'line-cap': 'round', 'z-index': 5,
            'line-color': function (e) { return tierColor(e.data('dependency')); },
            'target-arrow-color': function (e) { return tierColor(e.data('dependency')); },
            'target-arrow-shape': 'triangle', 'arrow-scale': 0.8,
            'label': 'data(label)', 'font-size': '11px', 'font-family': 'Roboto, system-ui, sans-serif', 'font-weight': 700,
            'color': '#64748b', 'text-opacity': 0.55, 'text-background-color': '#ffffff', 'text-background-opacity': 0.8,
            'text-background-padding': '2px', 'text-background-shape': 'roundrectangle', 'text-margin-y': -6,
            'transition-property': 'opacity, width', 'transition-duration': '0.15s'
          }
        },
        { selector: 'edge[source = target]', style: { 'curve-style': 'bezier', 'loop-direction': '-45deg', 'loop-sweep': '-90deg', 'control-point-step-size': 90 } },
        { selector: 'edge[arrow = "bi"]', style: { 'source-arrow-shape': 'triangle', 'source-arrow-color': function (e) { return tierColor(e.data('dependency')); } } },
        { selector: 'edge.hover', style: { 'width': 3, 'text-opacity': 1, 'z-index': 50 } },
        { selector: 'edge.dimmed', style: { 'opacity': 0.16 } }
      ],
      layout: { name: 'preset', fit: false, animate: false }
    });

    cyInstance.one('layoutstop', fitAll);
    cyInstance.ready(fitAll);
    resizeHandler = scheduleFit;
    window.addEventListener('resize', resizeHandler);

    cyInstance.on('tap', 'edge', function (evt) { isolateRule(evt.target); });
    cyInstance.on('tap', 'node', function (evt) { focusPolicy(evt.target); });

    cyInstance.on('mouseover', 'node', function (evt) {
      if (isolated) { return; }
      var n = evt.target;
      cyInstance.elements().addClass('dimmed');
      n.closedNeighborhood().removeClass('dimmed');
      n.addClass('hover');
      cyInstance.container().style.cursor = 'pointer';
    });
    cyInstance.on('mouseout', 'node', function (evt) {
      evt.target.removeClass('hover');
      if (!isolated) { cyInstance.elements().removeClass('dimmed'); }
      cyInstance.container().style.cursor = 'default';
    });
    cyInstance.on('mouseover', 'edge', function (evt) {
      if (isolated) { return; }
      var e = evt.target;
      cyInstance.elements().addClass('dimmed');
      e.union(e.connectedNodes()).removeClass('dimmed');
      e.addClass('hover');
      cyInstance.container().style.cursor = 'pointer';
    });
    cyInstance.on('mouseout', 'edge', function (evt) {
      evt.target.removeClass('hover');
      if (!isolated) { cyInstance.elements().removeClass('dimmed'); }
      cyInstance.container().style.cursor = 'default';
    });

    cyInstance.on('tap', function (evt) {
      if (evt.target === cyInstance && isolated) { restore(); }
    });
  }

  function initDependencyGraph() {
    var container = document.getElementById('dependency-graph');
    if (!container) { destroyGraph(); return; }
    if (typeof cytoscape === 'undefined') { return; }
    var dataUrl = new URL('../data/graph-data.json', window.location.href).href;
    fetch(dataUrl)
      .then(function (r) { if (!r.ok) { throw new Error('graph data'); } return r.json(); })
      .then(renderGraph)
      .catch(function () { destroyGraph(); });
  }

  function boot() {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', initDependencyGraph);
    } else {
      initDependencyGraph();
    }
    if (typeof document$ !== 'undefined' && document$.subscribe) {
      document$.subscribe(function () { initDependencyGraph(); });
    }
  }

  boot();
})();
